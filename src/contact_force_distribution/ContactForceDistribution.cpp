/*
 * ContactForceDistribution.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/contact_force_distribution/ContactForceDistribution.hpp"
#include <Eigen/Geometry>
#include "LinearAlgebra.hpp"
#include "sm/numerical_comparisons.hpp"
#include "OoqpEigenInterface.hpp"
#include "QuadraticProblemFormulation.hpp"
#include "Logger.hpp"

using namespace std;
using namespace Eigen;
using namespace robotUtils;
using namespace robotModel;
using namespace sm;

namespace loco {

ContactForceDistribution::ContactForceDistribution(robotModel::RobotModel* robotModel)
    : ContactForceDistributionBase(robotModel)
{
  for(auto leg : Legs()) { legStatuses_[leg] = LegStatus(); }
  resetFootLoadFactors();
}

ContactForceDistribution::~ContactForceDistribution()
{

}

bool ContactForceDistribution::loadParameters()
{
  // TODO Replace this with proper parameters loading (XML)
//  virtualForceWeights_ << 1.0, 1.0, 1.0, 10.0, 10.0, 5.0;
  virtualForceWeights_ << 1.0, 1.0, 0.1, 10.0, 10.0, 5.0;
  groundForceWeight_ = 0.00001;
  minimalNormalGroundForce_ = 2.0;
  frictionCoefficient_ = 0.7; // 0.8
  return ContactForceDistributionBase::loadParameters();
}

bool ContactForceDistribution::addToLogger()
{
  // TODO Is this already implemented somewhere else?
  for(const auto& leg : Legs()) {
    string name = "contact_force_" + legNamesShort[leg];
    VectorCF& contactForce = legStatuses_[leg].effectiveContactForce_;
    addEigenVector3ToLog(contactForce, name, "N", true);
  }

  updateLoggerData();

  return ContactForceDistributionBase::addToLogger();
}

bool ContactForceDistribution::changeLegLoad(const Legs& leg,
                                             const double& loadFactor)
{
  if (robotModel_->contacts().getCA()(legIndeces[leg]) == 1)
  {
    legStatuses_[leg].loadFactor_ = loadFactor;
    return true;
  }

  return false;
}

bool ContactForceDistribution::computeForceDistribution(
    const Eigen::Vector3d& virtualForce,
    const Eigen::Vector3d& virtualTorque)
{
  if(!checkIfParametersLoaded()) return false;
  if(!checkIfTerrainSet()) return false;

  resetConstraints();
  prepareLegLoading();

  if (nLegsInStance_ > 0)
  {
    prepareOptimization(virtualForce, virtualTorque);
    getTerrainNormals();

    // TODO Move these function calls to a separate method which can be overwritten
    // to have different contact force distributions, or let them be activated via parameters.
    addMinimalForceConstraints();
    addFrictionConstraints();

    // Has to be called as last
    addDesiredLegLoadConstraints();
    isForceDistributionComputed_ = solveOptimization();
  }

  resetFootLoadFactors();
  if (isLogging_) updateLoggerData();
  return isForceDistributionComputed_;
}

bool ContactForceDistribution::prepareLegLoading()
{
  nLegsInStance_ = 0;

  for (auto& legStatus : legStatuses_)
  {
    legStatus.second.loadFactor_ = legStatus.second.loadFactor_
        * robotModel_->contacts().getCA().cast<double>()(legIndeces[legStatus.first]);

    if (sm::definitelyGreaterThan(legStatus.second.loadFactor_, 0.0))
    {
      legStatus.second.isInStance_ = true;
      legStatus.second.indexInStanceLegList_ = nLegsInStance_;
      legStatus.second.startIndexInVectorX_ = legStatus.second.indexInStanceLegList_ * nTranslationalDofPerFoot_;
      nLegsInStance_++;

      if (sm::definitelyLessThan(legStatus.second.loadFactor_,1.0))
        legStatus.second.isLoadConstraintActive_ = true;
    }
    else
    {
      legStatus.second.isInStance_ = false;
      legStatus.second.isLoadConstraintActive_ = false;
    }
  }

  return true;
}

bool ContactForceDistribution::prepareOptimization(
    const Eigen::Vector3d& virtualForce,
    const Eigen::Vector3d& virtualTorque)
{
  n_ = nTranslationalDofPerFoot_ * nLegsInStance_;

  /*
   * Finds x that minimizes f = (Ax-b)' S (Ax-b) + x' W x, such that Cx = c and d <= Dx <= f.
   */
  b_.resize(nElementsVirtualForceTorqueVector_);
  b_.segment(0, virtualForce.size()) = virtualForce;
  b_.segment(virtualForce.size(), virtualTorque.size()) = virtualTorque;

  S_ = virtualForceWeights_.asDiagonal();

  A_.resize(nElementsVirtualForceTorqueVector_, n_);
  A_.setZero();
  A_.middleRows(0, nTranslationalDofPerFoot_) = (Matrix3d::Identity().replicate(1, nLegsInStance_)).sparseView();

  MatrixXd A_bottomMatrix(3, n_); // TODO replace 3 with nTranslationalDofPerFoot_
  for (auto& legStatus : legStatuses_)
  {
    if (legStatus.second.isInStance_)
    {
      VectorP r = robotModel_->kin().getJacobianTByLeg_Base2Foot_CSmb(legIndeces[legStatus.first])->getPos();
      A_bottomMatrix.block(0, legStatus.second.indexInStanceLegList_* r.size(), r.size(), r.size()) = getSkewMatrixFromVector(r);
    }
  }
  A_.middleRows(nTranslationalDofPerFoot_, A_bottomMatrix.rows()) = A_bottomMatrix.sparseView();

  W_.setIdentity(nTranslationalDofPerFoot_ * nLegsInStance_);
  W_ = W_ * groundForceWeight_;

  return true;
}

bool ContactForceDistribution::getTerrainNormals()
{
  if (!checkIfTerrainSet()) return false;

  for (auto& legStatus : legStatuses_)
  {
    if (legStatus.second.isInStance_)
    {
      VectorP footPosition = robotModel_->kin().getJacobianTByLeg_World2Foot_CSw(legIndeces[legStatus.first])->getPos();
      terrain_->getNormal(footPosition, legStatus.second.terrainNormalInWorldFrame_);
    }

    legStatus.second.terrainNormalInBaseFrame_ =
        robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA()
            * legStatus.second.terrainNormalInWorldFrame_;
  }

  return true;
}

bool ContactForceDistribution::addMinimalForceConstraints()
{
  /* We want each stance leg to have a minimal force in the normal direction to the ground:
   * n.f_i >= n.f_min with n.f_i the normal component of contact force.
   */
  int rowIndex = D_.rows();
  Eigen::SparseMatrix<double, Eigen::RowMajor> D_temp(D_);  // TODO replace with conservativeResize (available in Eigen 3.2)
  D_.resize(rowIndex + nLegsInStance_, n_);
  D_.middleRows(0, D_temp.rows()) = D_temp;
  d_.conservativeResize(rowIndex + nLegsInStance_);
  f_.conservativeResize(rowIndex + nLegsInStance_);

  for (auto& legStatus : legStatuses_)
  {
    if (legStatus.second.isInStance_)
    {
      MatrixXd D_row = MatrixXd::Zero(1, n_);
      D_row.block(0, legStatus.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_)
        = legStatus.second.terrainNormalInBaseFrame_.transpose();
      D_.middleRows(rowIndex, 1) = D_row.sparseView();
      d_(rowIndex) = minimalNormalGroundForce_;
      f_(rowIndex) = std::numeric_limits<double>::max();
      rowIndex++;
    }
  }

  return true;
}

bool ContactForceDistribution::addFrictionConstraints()
{
  /* For each tangent direction t, we want: -mu * n.f_i <= t.f_i <= mu * n.f_i
   * with n.f_i the normal component of contact force of leg i, t.f_i the tangential component).
   * This is equivalent to the two constraints: mu * n.f_i >= -t.f_i and mu * n.f_i >= t * f_i,
   * and equivalently mu * n.f_i + t.f_i >=0 and mu * n.f_i - t.f_i >= 0.
   * We have to define these constraints for both tangential directions (approximation of the
   * friction cone).
   */
  int nDirections = 4;
  int nConstraints = nDirections * nLegsInStance_;
  int rowIndex = D_.rows();
  Eigen::SparseMatrix<double, Eigen::RowMajor> D_temp(D_); // TODO replace with conservativeResize (available in Eigen 3.2)
  D_.resize(rowIndex + nConstraints, n_);
  D_.middleRows(0, D_temp.rows()) = D_temp;
  d_.conservativeResize(rowIndex + nConstraints);
  f_.conservativeResize(rowIndex + nConstraints);

  for (auto& legStatus : legStatuses_)
  {
    if (legStatus.second.isInStance_)
    {
      MatrixXd D_rows = MatrixXd::Zero(nDirections, n_);

      Vector3d& normalDirection = legStatus.second.terrainNormalInBaseFrame_;

      // The choose the first tangential to lie in the XZ-plane of the base frame.
      // This is the same as the requirement as
      // 1) firstTangential perpendicular to normalDirection,
      // 2) firstTangential perpendicular to normal of XZ-plane of the base frame,
      // 3) firstTangential has unit norm.
      Vector3d firstTangential = normalDirection.cross(Vector3d::UnitY()).normalized();

      // The second tangential is perpendicular to the normal and the first tangential.
      Vector3d secondTangential = normalDirection.cross(firstTangential).normalized();

      // First tangential, positive
      D_rows.block(0, legStatus.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_) =
          frictionCoefficient_ * normalDirection.transpose() + firstTangential.transpose();
      // First tangential, negative
      D_rows.block(1, legStatus.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_) =
          frictionCoefficient_ * normalDirection.transpose() - firstTangential.transpose();
      // Second tangential, positive
      D_rows.block(2, legStatus.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_) =
          frictionCoefficient_ * normalDirection.transpose() + secondTangential.transpose();
      // Second tangential, negative
      D_rows.block(3, legStatus.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_) =
          frictionCoefficient_ * normalDirection.transpose() - secondTangential.transpose();

      D_.middleRows(rowIndex, nDirections) = D_rows.sparseView();
      d_.segment(rowIndex, nDirections) =  VectorXd::Constant(nDirections, 0.0);
      f_.segment(rowIndex, nDirections) = VectorXd::Constant(nDirections, std::numeric_limits<double>::max());

      rowIndex = rowIndex + nDirections;
    }
  }

  return true;
}

bool ContactForceDistribution::addDesiredLegLoadConstraints()
{
  /* For each leg with user defined load constraints, we add equality constraints of the form
   * f_i = loadFactor * f_i_previous, with f_i the contact force of leg i and f_i_previous
   * the contact force of leg i at the optimization without user defined leg load constraints.
   */
  int nLegsWithLoadConstraintActive = 0;
  for (auto& legStatus : legStatuses_)
  {
    if (legStatus.second.isLoadConstraintActive_)
      nLegsWithLoadConstraintActive++;
  }
  if (nLegsWithLoadConstraintActive == 0) return true; // No need to have equality constraints

  solveOptimization(); // Solving once without equality constraints

  int nConstraints = nTranslationalDofPerFoot_ * nLegsWithLoadConstraintActive;
  int rowIndex = C_.rows();
  Eigen::SparseMatrix<double, Eigen::RowMajor> C_temp(C_);  // TODO replace with conservativeResize (available in Eigen 3.2)
  C_.resize(rowIndex + nConstraints, n_);
  C_.middleRows(0, C_temp.rows()) = C_temp;
  c_.conservativeResize(rowIndex + nConstraints);

  for (auto& legStatus : legStatuses_)
  {
    if (legStatus.second.isLoadConstraintActive_)
    {
      int m = nTranslationalDofPerFoot_;
      MatrixXd C_rows = MatrixXd::Zero(m, n_);
      C_rows.block(0, legStatus.second.startIndexInVectorX_, m, m) = MatrixXd::Identity(m, m);
      C_.middleRows(rowIndex, m) = C_rows.sparseView();
      Eigen::Matrix<double, nTranslationalDofPerFoot_, 1> fullForce = x_.segment(legStatus.second.startIndexInVectorX_, nTranslationalDofPerFoot_);
      c_.segment(rowIndex, m) =  legStatus.second.loadFactor_ * fullForce;
      rowIndex = rowIndex + m;
    }
  }

  return true;
}

bool ContactForceDistribution::solveOptimization()
{
  return ooqpei::QuadraticProblemFormulation::solve(A_, S_, b_, W_, C_, c_, D_, d_, f_, x_);
}

bool ContactForceDistribution::resetConstraints()
{
  D_.resize(0, 0);
  d_.resize(0);
  f_.resize(0);
  C_.resize(0, 0);
  c_.resize(0);
  return true;
}

bool ContactForceDistribution::resetFootLoadFactors()
{
  for (auto& legStatus : legStatuses_)
  {
    legStatus.second.loadFactor_ = 1.0;
    legStatus.second.isLoadConstraintActive_ = false;
  }
  return true;
}

bool ContactForceDistribution::getForceForLeg(
    const Legs& leg, robotModel::VectorCF& force)
{
  if (!checkIfForceDistributionComputed()) return false;

  if (legStatuses_[leg].isInStance_)
  {
    // The forces we computed here are actually the ground reaction forces,
    // so the stance legs should push the ground by the opposite amount.
    force = -x_.segment(legStatuses_[leg].startIndexInVectorX_, nTranslationalDofPerFoot_);
    return true;
  }

  return false;
}

bool ContactForceDistribution::getNetForceAndTorqueOnBase(
    Eigen::Vector3d& netForce, Eigen::Vector3d& netTorque)
{
  if (!checkIfForceDistributionComputed()) return false;

  Eigen::Matrix<double, nElementsVirtualForceTorqueVector_, 1> stackedNetForceAndTorque;
  stackedNetForceAndTorque = A_ * x_;
  netForce = stackedNetForceAndTorque.head(3);
  netTorque = stackedNetForceAndTorque.tail(3);

  return true;
}

bool ContactForceDistribution::updateLoggerData()
{
  if (!isLogging_) return false;

  for(const auto& leg : Legs()) {
    legStatuses_[leg].effectiveContactForce_ = robotModel_->contacts().getCP(legIndeces[leg])->getForce();
  }

  return true;
}

} /* namespace loco */
