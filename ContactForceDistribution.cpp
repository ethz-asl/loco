/*
 * ContactForceDistribution.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "ContactForceDistribution.hpp"
#include <Eigen/Geometry>
#include "LinearAlgebra.hpp"
#include "NumericalComparison.hpp"
#include "OoqpInterface.hpp"
#include "QuadraticProblemFormulation.hpp"
#include "Logger.hpp"

using namespace std;
using namespace Eigen;
using namespace robotUtils;
using namespace robotModel;

namespace robotController {

ContactForceDistribution::ContactForceDistribution(robotModel::RobotModel* robotModel)
{
  robotModel_ = robotModel;
  for(auto leg : Legs()) { legStatuses_[leg] = LegStatus(); }
  resetFootLoadFactors();
}

ContactForceDistribution::~ContactForceDistribution()
{
}

bool ContactForceDistribution::loadParameters()
{
  // TODO Replace this with proper parameters loading (XML)
  virtualForceWeights_ << 1.0, 1.0, 1.0, 10.0, 10.0, 5.0;
  groundForceWeight_ = 0.00001;
  minimalNormalGroundForce_ = 2.0;
  frictionCoefficient_ = 0.8;
  isParametersLoaded_ = true;
  return true;
}

bool ContactForceDistribution::isParametersLoaded() const
{
  if (isParametersLoaded_) return true;

  cout << "Contact force distribution parameters are not loaded." << endl; // TODO use warning output
  return false;
}

bool ContactForceDistribution::isForceDistributionComputed() const
{
  if (isForceDistributionComputed_) return true;

  cout << "Contact force distribution is not computed yet or was unsuccessful." << endl; // TODO use warning output
  return false;
}

bool ContactForceDistribution::addToLogger()
{
  for(const auto& leg : Legs()) {
    string name = "contact_force_" + legNamesShort[leg];
    VectorCF& contactForce = legStatuses_[leg].effectiveContactForce_;
    addEigenVector3ToLog(contactForce, name, "N", true);
  }

  updateLoggerData();
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
  resetConstraints();
  prepareDesiredLegLoading();
  if (nLegsInStance_ > 0)
  {
    prepareOptimization(virtualForce, virtualTorque);

    // Add, remove or switch constraints
    addMinimalForceConstraints();
    addFrictionConstraints();

    // Has to be called at last
    addDesiredLegLoadConstraints();
    isForceDistributionComputed_ = solveOptimization();
  }
  resetFootLoadFactors();
  updateLoggerData();
  return isForceDistributionComputed_;
}

bool ContactForceDistribution::prepareDesiredLegLoading()
{
  nLegsInStance_ = 0;

  for (auto& legStatus : legStatuses_)
  {
    legStatus.second.loadFactor_ = legStatus.second.loadFactor_
        * robotModel_->contacts().getCA().cast<double>()(legIndeces[legStatus.first]);

    if (NumericalComparison::definitelyGreaterThan(legStatus.second.loadFactor_, 0.0))
    {
      legStatus.second.isInStance_ = true;
      legStatus.second.indexInStanceLegList_ = nLegsInStance_;
      legStatus.second.startIndexInVectorX_ = legStatus.second.indexInStanceLegList_ * nTranslationalDofPerFoot_;
      nLegsInStance_++;

      if (NumericalComparison::definitelyLessThan(legStatus.second.loadFactor_,1.0))
        legStatus.second.isLoadConstraintActive_ = true;
    }
    else
    {
      legStatus.second.isInStance_ = false;
      legStatus.second.isLoadConstraintActive_ = false;
    }
  }
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

bool ContactForceDistribution::addMinimalForceConstraints()
{
  /* We want each stance leg to have a minimal force in the normal direction to the ground:
   * n.f_i >= n.f_min with n.f_i the normal component of contact force.
   */
  MatrixA transformationFromWorldToBase = robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA();
  Vector3d normalDirection = transformationFromWorldToBase * Vector3d::UnitZ();

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
      D_row.block(0, legStatus.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_) = normalDirection.transpose();
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
  MatrixA transformationFromWorldToBase = robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA();
  Vector3d normalDirection = transformationFromWorldToBase * Vector3d::UnitZ();
  Vector3d firstTangential = transformationFromWorldToBase * Vector3d::UnitX();
  Vector3d secondTangential = normalDirection.cross(firstTangential);

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
  int nLegsWithLoadConstraintActive = 0;
  for (auto& legStatus : legStatuses_)
  {
    if (legStatus.second.isLoadConstraintActive_)
      nLegsWithLoadConstraintActive++;
  }
  if (nLegsWithLoadConstraintActive == 0) return true; // No need to have equality constraints

  solveOptimization(); // Solving ones without equality constraints

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
  return QuadraticProblemFormulation::solve(A_, S_, b_, W_, C_, c_, D_, d_, f_, x_);
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
  if (!isForceDistributionComputed()) return false;

  force = VectorCF();

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
  if (!isForceDistributionComputed()) return false;

  Eigen::Matrix<double, nElementsVirtualForceTorqueVector_, 1> stackedNetForceAndTorque;
  stackedNetForceAndTorque = A_ * x_;
  netForce = stackedNetForceAndTorque.head(3);
  netTorque = stackedNetForceAndTorque.tail(3);

  return true;
}

bool ContactForceDistribution::updateLoggerData()
{
  for(const auto& leg : Legs()) {
    legStatuses_[leg].effectiveContactForce_ = robotModel_->contacts().getCP(legIndeces[leg])->getForce();
  }

  return true;
}

} /* namespace robotController */
