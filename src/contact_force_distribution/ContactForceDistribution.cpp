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

ContactForceDistribution::ContactForceDistribution(std::shared_ptr<LegGroup> legs, std::shared_ptr<robotTerrain::TerrainBase> terrain)
    : ContactForceDistributionBase(legs, terrain)
{
  for(auto leg : *legs_) { legInfos_[leg] = LegInfo(); }
}

ContactForceDistribution::~ContactForceDistribution()
{

}

bool ContactForceDistribution::loadParameters(const TiXmlHandle& handle)
{
  // TODO Replace this with proper parameters loading (XML)
//  virtualForceWeights_ << 1.0, 1.0, 1.0, 10.0, 10.0, 5.0;
  virtualForceWeights_ << 1.0, 1.0, 0.1, 10.0, 10.0, 5.0;
  groundForceWeight_ = 0.00001;
  minimalNormalGroundForce_ = 2.0;
  frictionCoefficient_ = 2.0;
  return ContactForceDistributionBase::loadParameters(handle);
}

bool ContactForceDistribution::addToLogger()
{
  // TODO Is this already implemented somewhere else?
//  for(const auto& leg : Legs()) {
//    string name = "contact_force_" + legNamesShort[leg];
//    VectorCF& contactForce = legInfos_[leg].effectiveContactForce_;
//    addEigenVector3ToLog(contactForce, name, "N", true);
//  }

  updateLoggerData();

  return ContactForceDistributionBase::addToLogger();
}

bool ContactForceDistribution::computeForceDistribution(
    const Force& virtualForce,
    const Torque& virtualTorque)
{
  if(!checkIfParametersLoaded()) return false;

  resetOptimization();
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

    if (isForceDistributionComputed_)
    {
      computeJointTorques();
    }
  }

  if (isLogging_) updateLoggerData();
  return isForceDistributionComputed_;
}

bool ContactForceDistribution::prepareLegLoading()
{
  nLegsInStance_ = 0;

  for (auto& legInfo : legInfos_)
  {
    // TODO isAndShouldBeGrounded() is this the correct one?
    if (sm::definitelyGreaterThan(legInfo.first->getDesiredLoadFactor(), 0.0) && legInfo.first->isAndShouldBeGrounded())
    {
      legInfo.second.isPartOfOptimization_ = true;
      legInfo.second.isLoadConstraintActive_ = false;
      legInfo.second.indexInStanceLegList_ = nLegsInStance_;
      legInfo.second.startIndexInVectorX_ = legInfo.second.indexInStanceLegList_ * nTranslationalDofPerFoot_;
      nLegsInStance_++;

      if (sm::definitelyLessThan(legInfo.first->getDesiredLoadFactor(), 1.0))
        legInfo.second.isLoadConstraintActive_ = true;
    }
    else
    {
      legInfo.second.isPartOfOptimization_ = false;
      legInfo.second.isLoadConstraintActive_ = false;
    }
  }

  return true;
}

bool ContactForceDistribution::prepareOptimization(
    const Force& virtualForce,
    const Torque& virtualTorque)
{
  n_ = nTranslationalDofPerFoot_ * nLegsInStance_;

  /*
   * Finds x that minimizes f = (Ax-b)' S (Ax-b) + x' W x, such that Cx = c and d <= Dx <= f.
   */
  b_.resize(nElementsVirtualForceTorqueVector_);
  b_.segment(0, virtualForce.toImplementation().size()) = virtualForce.toImplementation();
  b_.segment(virtualForce.toImplementation().size(), virtualTorque.toImplementation().size()) = virtualTorque.toImplementation();

  S_ = virtualForceWeights_.asDiagonal();

  A_.resize(nElementsVirtualForceTorqueVector_, n_);
  A_.setZero();
  A_.middleRows(0, nTranslationalDofPerFoot_) = (Matrix3d::Identity().replicate(1, nLegsInStance_)).sparseView();

  MatrixXd A_bottomMatrix(3, n_); // TODO replace 3 with nTranslationalDofPerFoot_
  for (auto& legInfo : legInfos_)
  {
    if (legInfo.second.isPartOfOptimization_)
    {
      const Vector3d& r = legInfo.first->getBaseToFootPositionInBaseFrame().toImplementation();
      A_bottomMatrix.block(0, legInfo.second.indexInStanceLegList_ * r.size(), r.size(), r.size()) =
          getSkewMatrixFromVector(r);
    }
  }
  A_.middleRows(nTranslationalDofPerFoot_, A_bottomMatrix.rows()) = A_bottomMatrix.sparseView();

  W_.setIdentity(nTranslationalDofPerFoot_ * nLegsInStance_);
  W_ = W_ * groundForceWeight_;

  return true;
}

bool ContactForceDistribution::getTerrainNormals()
{
  for (auto& legInfo : legInfos_)
  {
    if (legInfo.second.isPartOfOptimization_)
    {
      Vector3d surfaceNormal = Vector3d::Zero();
      if (legInfo.first->isGrounded()) {
        terrain_->getNormal(legInfo.first->getWorldToFootPositionInWorldFrame().toImplementation(), surfaceNormal);
      }
      legInfo.first->setSurfaceNormal(surfaceNormal);
    }
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

  for (auto& legInfo : legInfos_)
  {
    if (legInfo.second.isPartOfOptimization_)
    {
      MatrixXd D_row = MatrixXd::Zero(1, n_);
      D_row.block(0, legInfo.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_)
        = legInfo.first->getSurfaceNormal().transpose();
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

  for (auto& legInfo : legInfos_)
  {
    if (legInfo.second.isPartOfOptimization_)
    {
      MatrixXd D_rows = MatrixXd::Zero(nDirections, n_);

      const Vector3d& normalDirection = legInfo.first->getSurfaceNormal();

      // The choose the first tangential to lie in the XZ-plane of the base frame.
      // This is the same as the requirement as
      // 1) firstTangential perpendicular to normalDirection,
      // 2) firstTangential perpendicular to normal of XZ-plane of the base frame,
      // 3) firstTangential has unit norm.
      Vector3d firstTangential = normalDirection.cross(Vector3d::UnitY()).normalized();

      // The second tangential is perpendicular to the normal and the first tangential.
      Vector3d secondTangential = normalDirection.cross(firstTangential).normalized();

      // First tangential, positive
      D_rows.block(0, legInfo.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_) =
          frictionCoefficient_ * normalDirection.transpose() + firstTangential.transpose();
      // First tangential, negative
      D_rows.block(1, legInfo.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_) =
          frictionCoefficient_ * normalDirection.transpose() - firstTangential.transpose();
      // Second tangential, positive
      D_rows.block(2, legInfo.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_) =
          frictionCoefficient_ * normalDirection.transpose() + secondTangential.transpose();
      // Second tangential, negative
      D_rows.block(3, legInfo.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_) =
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
  /*
   * The contact force distribution is calculated
   * twice, once without the load factor equality constraint and then
   * including the equality constraint.
   * For each leg with user defined load constraints, we add equality constraints of the form
   * f_i = loadFactor * f_i_previous, with f_i the contact force of leg i and f_i_previous
   * the contact force of leg i at the optimization without user defined leg load constraints.
   */
  int nLegsWithLoadConstraintActive = 0;
  for (auto& legStatus : legInfos_)
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

  for (auto& legInfo : legInfos_)
  {
    if (legInfo.second.isLoadConstraintActive_)
    {
      int m = nTranslationalDofPerFoot_;
      MatrixXd C_rows = MatrixXd::Zero(m, n_);
      C_rows.block(0, legInfo.second.startIndexInVectorX_, m, m) = MatrixXd::Identity(m, m);
      C_.middleRows(rowIndex, m) = C_rows.sparseView();
      Eigen::Matrix<double, nTranslationalDofPerFoot_, 1> fullForce = x_.segment(legInfo.second.startIndexInVectorX_, nTranslationalDofPerFoot_);
      c_.segment(rowIndex, m) =  legInfo.first->getDesiredLoadFactor() * fullForce;
      rowIndex = rowIndex + m;
    }
  }

  return true;
}

bool ContactForceDistribution::solveOptimization()
{
  if (!ooqpei::QuadraticProblemFormulation::solve(A_, S_, b_, W_, C_, c_, D_, d_, f_, x_))
    return false;

  for (auto& legInfo : legInfos_)
  {
    if (legInfo.second.isPartOfOptimization_)
    {
      // The forces we computed here are actually the ground reaction forces,
      // so the stance legs should push the ground by the opposite amount.
      legInfo.second.desiredContactForce_ =
          Force(-x_.segment(legInfo.second.startIndexInVectorX_, nTranslationalDofPerFoot_));
    }
  }

  return true;
}

bool ContactForceDistribution::computeJointTorques()
{
  const int nDofPerLeg = 3; // TODO move to robot commons
  const int nDofPerContactPoint = 3; // TODO move to robot commons

  for (auto& legInfo : legInfos_)
  {
    if (legInfo.second.isPartOfOptimization_)
    {
      LegBase::TranslationJacobian jacobian = legInfo.first->getTranslationJacobianFromBaseToFootInBaseFrame();
      Force contactForce = legInfo.second.desiredContactForce_;
      LegBase::JointTorques jointTorques = LegBase::JointTorques(jacobian.transpose() * contactForce.toImplementation());
      legInfo.first->setDesiredJointTorques(jointTorques);
    }
    else
    {
      legInfo.first->setDesiredJointTorques(LegBase::JointTorques::Zero());
    }
  }

  return true;
}

bool ContactForceDistribution::resetOptimization()
{
  isForceDistributionComputed_ = false;

  D_.resize(0, 0);
  d_.resize(0);
  f_.resize(0);
  C_.resize(0, 0);
  c_.resize(0);

  for (auto& legInfo : legInfos_)
  {
    legInfo.second.desiredContactForce_.setZero();
  }

  return true;
}

//bool ContactForceDistribution::getForceForLeg(
//    const Legs& leg, robotModel::VectorCF& force)
//{
//  if (!checkIfForceDistributionComputed()) return false;
//
//  if (legInfos_[leg].isInStance_)
//  {
//    // The forces we computed here are actually the ground reaction forces,
//    // so the stance legs should push the ground by the opposite amount.
//    force = -x_.segment(legInfos_[leg].startIndexInVectorX_, nTranslationalDofPerFoot_);
//    return true;
//  }
//
//  return false;
//}

bool ContactForceDistribution::getNetForceAndTorqueOnBase(
    Force& netForce, Torque& netTorque)
{
  if (!checkIfForceDistributionComputed()) return false;

  Eigen::Matrix<double, nElementsVirtualForceTorqueVector_, 1> stackedNetForceAndTorque;
  stackedNetForceAndTorque = A_ * x_;
  netForce = Force(stackedNetForceAndTorque.head(3));
  netTorque = Torque(stackedNetForceAndTorque.tail(3));

  return true;
}

bool ContactForceDistribution::updateLoggerData()
{
  if (!isLogging_) return false;

  return true;
}

} /* namespace loco */
