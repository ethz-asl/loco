/*
 * ContactForceDistribution.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/contact_force_distribution/ContactForceDistribution.hpp"
#include "loco/common/LegLinkGroup.hpp"

#include <Eigen/Geometry>
#include "robotUtils/math/LinearAlgebra.hpp"
//#include "sm/numerical_comparisons.hpp"
#include "OoqpEigenInterface.hpp"
#include "QuadraticProblemFormulation.hpp"
#include "robotUtils/loggers/logger.hpp"

#include "loco/temp_helpers/math.hpp"


using namespace std;
using namespace Eigen;
//using namespace sm;

namespace loco {

ContactForceDistribution::ContactForceDistribution(std::shared_ptr<TorsoBase> torso, std::shared_ptr<LegGroup> legs, std::shared_ptr<loco::TerrainModelBase> terrain)
    : ContactForceDistributionBase(torso, legs, terrain)
{
  for(auto leg : *legs_) {
    legInfos_[leg] = LegInfo();
  }
}

ContactForceDistribution::~ContactForceDistribution()
{

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

  return true;
}

bool ContactForceDistribution::computeForceDistribution(
    const Force& virtualForceInBaseFrame,
    const Torque& virtualTorqueInBaseFrame)
{
  if(!checkIfParametersLoaded()) return false;

  resetOptimization();
  prepareLegLoading();

  if (nLegsInForceDistribution_ > 0)
  {
    prepareOptimization(virtualForceInBaseFrame, virtualTorqueInBaseFrame);

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
  else
  {
    // No leg is part of the force distribution
    isForceDistributionComputed_ = true;
    computeJointTorques();
  }

  if (isLogging_) updateLoggerData();
  return isForceDistributionComputed_;
}

bool ContactForceDistribution::prepareLegLoading()
{
  nLegsInForceDistribution_ = 0;

  for (auto& legInfo : legInfos_)
  {
//    if (sm::definitelyGreaterThan(legInfo.first->getDesiredLoadFactor(), 0.0) && legInfo.first->isAndShouldBeGrounded())
    if ((legInfo.first->getDesiredLoadFactor() > 0.0) && legInfo.first->isSupportLeg()) // get rid of sm dependency
    {
      legInfo.second.isPartOfForceDistribution_ = true;
      legInfo.second.isLoadConstraintActive_ = false;
      legInfo.second.indexInStanceLegList_ = nLegsInForceDistribution_;
      legInfo.second.startIndexInVectorX_ = legInfo.second.indexInStanceLegList_ * nTranslationalDofPerFoot_;
      nLegsInForceDistribution_++;

//      if (sm::definitelyLessThan(legInfo.first->getDesiredLoadFactor(), 1.0))
      if (legInfo.first->getDesiredLoadFactor() < 1.0)
        legInfo.second.isLoadConstraintActive_ = true;
    }
    else
    {
      legInfo.second.isPartOfForceDistribution_ = false;
      legInfo.second.isLoadConstraintActive_ = false;
    }
  }

  return true;
}

bool ContactForceDistribution::prepareOptimization(
    const Force& virtualForce,
    const Torque& virtualTorque)
{
  n_ = nTranslationalDofPerFoot_ * nLegsInForceDistribution_;

  /*
   * Finds x that minimizes f = (Ax-b)' S (Ax-b) + x' W x, such that Cx = c and d <= Dx <= f.
   */
  b_.resize(nElementsVirtualForceTorqueVector_);
  b_.segment(0, virtualForce.toImplementation().size()) = virtualForce.toImplementation();
  b_.segment(virtualForce.toImplementation().size(), virtualTorque.toImplementation().size()) = virtualTorque.toImplementation();

  S_ = virtualForceWeights_.asDiagonal();

  A_.resize(nElementsVirtualForceTorqueVector_, n_);
  A_.setZero();
  A_.middleRows(0, nTranslationalDofPerFoot_) = (Matrix3d::Identity().replicate(1, nLegsInForceDistribution_)).sparseView();

  MatrixXd A_bottomMatrix(3, n_); // TODO replace 3 with nTranslationalDofPerFoot_
  for (auto& legInfo : legInfos_)
  {
    if (legInfo.second.isPartOfForceDistribution_)
    {
      const Vector3d& r = legInfo.first->getPositionBaseToFootInBaseFrame().toImplementation();
      A_bottomMatrix.block(0, legInfo.second.indexInStanceLegList_ * r.size(), r.size(), r.size()) =
          getSkewMatrixFromVector(r);
    }
  }
  A_.middleRows(nTranslationalDofPerFoot_, A_bottomMatrix.rows()) = A_bottomMatrix.sparseView();

  W_.setIdentity(nTranslationalDofPerFoot_ * nLegsInForceDistribution_);
  W_ = W_ * groundForceWeight_;

  return true;
}



bool ContactForceDistribution::addMinimalForceConstraints()
{
  /* We want each stance leg to have a minimal force in the normal direction to the ground:
   * n.f_i >= n.f_min with n.f_i the normal component of contact force.
   */
  int rowIndex = D_.rows();
  Eigen::SparseMatrix<double, Eigen::RowMajor> D_temp(D_);  // TODO replace with conservativeResize (available in Eigen 3.2)
  D_.resize(rowIndex + nLegsInForceDistribution_, n_);
  D_.middleRows(0, D_temp.rows()) = D_temp;
  d_.conservativeResize(rowIndex + nLegsInForceDistribution_);
  f_.conservativeResize(rowIndex + nLegsInForceDistribution_);

  const RotationQuaternion& orientationWorldToBase = torso_->getMeasuredState().getOrientationWorldToBase();

  for (auto& legInfo : legInfos_)
  {
    if (legInfo.second.isPartOfForceDistribution_)
    {
      Position positionWorldToFootInWorldFrame = legInfo.first->getPositionWorldToFootInWorldFrame();
      Vector footContactNormalInWorldFrame;
      terrain_->getNormal(positionWorldToFootInWorldFrame, footContactNormalInWorldFrame);
      Vector footContactNormalInBaseFrame = orientationWorldToBase.rotate(footContactNormalInWorldFrame);

      MatrixXd D_row = MatrixXd::Zero(1, n_);
      D_row.block(0, legInfo.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_)
        = footContactNormalInBaseFrame.toImplementation().transpose();
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
  int nConstraints = nDirections * nLegsInForceDistribution_;
  int rowIndex = D_.rows();
  Eigen::SparseMatrix<double, Eigen::RowMajor> D_temp(D_); // TODO replace with conservativeResize (available in Eigen 3.2)
  D_.resize(rowIndex + nConstraints, n_);
  D_.middleRows(0, D_temp.rows()) = D_temp;
  d_.conservativeResize(rowIndex + nConstraints);
  f_.conservativeResize(rowIndex + nConstraints);

  const RotationQuaternion& orientationWorldToBase = torso_->getMeasuredState().getOrientationWorldToBase();
  const RotationQuaternion orientationControlToBase = torso_->getMeasuredState().getOrientationControlToBase();

  for (auto& legInfo : legInfos_)
  {
    if (legInfo.second.isPartOfForceDistribution_)
    {
      MatrixXd D_rows = MatrixXd::Zero(nDirections, n_);


      Position positionWorldToFootInWorldFrame = legInfo.first->getPositionWorldToFootInWorldFrame();
      Vector footContactNormalInWorldFrame;
      terrain_->getNormal(positionWorldToFootInWorldFrame, footContactNormalInWorldFrame);
      Vector footContactNormalInBaseFrame = orientationWorldToBase.rotate(footContactNormalInWorldFrame);

//      const Vector3d& normalDirection = legInfo.first->getFootContactNormalInWorldFrame().toImplementation();
      const Vector3d normalDirection = footContactNormalInBaseFrame.toImplementation();

      // for logging
      legInfo.second.normalDirectionOfFrictionPyramidInWorldFrame_ = loco::Vector(footContactNormalInWorldFrame);

      // The choose the first tangential to lie in the XZ-plane of the base frame.
      // This is the same as the requirement as
      // 1) firstTangential perpendicular to normalDirection,
      // 2) firstTangential perpendicular to normal of XZ-plane of the base frame,
      // 3) firstTangential has unit norm.
//      Vector3d firstTangential = normalDirection.cross(Vector3d::UnitY()).normalized();

      Vector3d vectorY = Vector3d::UnitY();
      Vector3d firstTangentialInBaseFrame = orientationControlToBase.rotate(vectorY);
      Vector3d firstTangential = normalDirection.cross(firstTangentialInBaseFrame).normalized();

      // logging
      legInfo.second.firstDirectionOfFrictionPyramidInWorldFrame_ = loco::Vector(orientationWorldToBase.inverseRotate(firstTangential));

      // The second tangential is perpendicular to the normal and the first tangential.
      Vector3d secondTangential = normalDirection.cross(firstTangential).normalized();

      // logging
      legInfo.second.secondDirectionOfFrictionPyramidInWorldFrame_ = loco::Vector(orientationWorldToBase.inverseRotate(secondTangential));

      // First tangential, positive
      D_rows.block(0, legInfo.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_) =
          legInfo.second.frictionCoefficient_ * normalDirection.transpose() + firstTangential.transpose();
      // First tangential, negative
      D_rows.block(1, legInfo.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_) =
          legInfo.second.frictionCoefficient_ * normalDirection.transpose() - firstTangential.transpose();
      // Second tangential, positive
      D_rows.block(2, legInfo.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_) =
          legInfo.second.frictionCoefficient_ * normalDirection.transpose() + secondTangential.transpose();
      // Second tangential, negative
      D_rows.block(3, legInfo.second.startIndexInVectorX_, 1, nTranslationalDofPerFoot_) =
          legInfo.second.frictionCoefficient_ * normalDirection.transpose() - secondTangential.transpose();

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

  // Finds x that minimizes (Ax-b)' S (Ax-b) + x' W x, such that Cx = c and d <= Dx <= f
  if (!ooqpei::QuadraticProblemFormulation::solve(A_, S_, b_, W_, C_, c_, D_, d_, f_, x_))
    return false;

  for (auto& legInfo : legInfos_)
  {
    if (legInfo.second.isPartOfForceDistribution_)
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
  const LinearAcceleration gravitationalAccelerationInWorldFrame = torso_->getProperties().getGravity();
  const LinearAcceleration gravitationalAccelerationInBaseFrame = torso_->getMeasuredState().getOrientationWorldToBase().rotate(gravitationalAccelerationInWorldFrame);


//  const int nDofPerLeg = 3; // TODO move to robot commons
//  const int nDofPerContactPoint = 3; // TODO move to robot commons

  for (auto& legInfo : legInfos_)
  {

    /*
     * Torque setpoints should be updated only is leg is support leg.
     */
    if  (legInfo.first->isSupportLeg()) {

      if (legInfo.second.isPartOfForceDistribution_)
      {
        LegBase::TranslationJacobian jacobian = legInfo.first->getTranslationJacobianFromBaseToFootInBaseFrame();

        Force contactForce = legInfo.second.desiredContactForce_;
        LegBase::JointTorques jointTorques = LegBase::JointTorques(jacobian.transpose() * contactForce.toImplementation());
  //      jointTorques += LegBase::JointTorques(torso_ Force(-torso_->getProperties().getMass() * gravitationalAccelerationInBaseFrame));
        /* gravity */
        for (auto link : *legInfo.first->getLinks()) {
          jointTorques -= LegBase::JointTorques( link->getTranslationJacobianBaseToCoMInBaseFrame().transpose() * Force(link->getMass() * gravitationalAccelerationInBaseFrame).toImplementation());
        }
        legInfo.first->setDesiredJointTorques(jointTorques);
      }
      else
      {
        /*
         * True if load factor is zero.
         */
        legInfo.first->setDesiredJointTorques(LegBase::JointTorques::Zero());
      }

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

double ContactForceDistribution::getGroundForceWeight() const {
  return groundForceWeight_;
}

double ContactForceDistribution::getMinimalNormalGroundForce() const {
  return minimalNormalGroundForce_;
}
double ContactForceDistribution::getVirtualForceWeight(int index) const {
  return virtualForceWeights_(index);
}

const Vector& ContactForceDistribution::getFirstDirectionOfFrictionPyramidInWorldFrame(LegBase* leg) const {
  return legInfos_.at(leg).firstDirectionOfFrictionPyramidInWorldFrame_;
}
const Vector& ContactForceDistribution::getSecondDirectionOfFrictionPyramidInWorldFrame(LegBase* leg) const {
  return legInfos_.at(leg).secondDirectionOfFrictionPyramidInWorldFrame_;
}
const Vector& ContactForceDistribution::getNormalDirectionOfFrictionPyramidInWorldFrame(LegBase* leg) const {
  return legInfos_.at(leg).normalDirectionOfFrictionPyramidInWorldFrame_;
}

double ContactForceDistribution::getFrictionCoefficient(LegBase* leg) const {
  return legInfos_.at(leg).frictionCoefficient_;
}

double ContactForceDistribution::getFrictionCoefficient(const LegBase* leg) const {
  return legInfos_.at(const_cast<LegBase*>(leg)).frictionCoefficient_;
}

double ContactForceDistribution::getFrictionCoefficient(int index) const {
  const LegBase* leg = legs_->getLeg(index);
  return legInfos_.at(const_cast<LegBase*>(leg)).frictionCoefficient_;
}


const ContactForceDistribution::LegInfo& ContactForceDistribution::getLegInfo(LegBase* leg) const {
  return legInfos_.at(leg);
}

bool ContactForceDistribution::setToInterpolated(const ContactForceDistributionBase& contactForceDistribution1, const ContactForceDistributionBase& contactForceDistribution2, double t) {
  const ContactForceDistribution& distribution1 = static_cast<const ContactForceDistribution&>(contactForceDistribution1);
  const ContactForceDistribution& distribution2 = static_cast<const ContactForceDistribution&>(contactForceDistribution2);

  if(!distribution1.checkIfParametersLoaded()) {
    return false;
  }
  if(!distribution2.checkIfParametersLoaded()) {
    return false;
  }


  for (auto leg : *legs_) {
    legInfos_.at(leg).frictionCoefficient_ = linearlyInterpolate(distribution1.getFrictionCoefficient(leg->getId()) , distribution2.getFrictionCoefficient(leg->getId()), 0.0, 1.0, t);
  }

  this->groundForceWeight_ = linearlyInterpolate(distribution1.getGroundForceWeight(), distribution2.getGroundForceWeight(), 0.0, 1.0, t);
  this->minimalNormalGroundForce_ = linearlyInterpolate(distribution1.getMinimalNormalGroundForce(), distribution2.getMinimalNormalGroundForce(), 0.0, 1.0, t);

  for (int i=0; i<(int)this->virtualForceWeights_.size(); i++) {
    this->virtualForceWeights_(i) = linearlyInterpolate(distribution1.getVirtualForceWeight(i), distribution2.getVirtualForceWeight(i), 0.0, 1.0, t);
  }
  return true;
}


bool ContactForceDistribution::loadParameters(const TiXmlHandle& handle)
{
  isParametersLoaded_ = false;

  TiXmlElement* element = handle.FirstChild("ContactForceDistribution").ToElement();
  if (!element) {
    printf("Could not find ContactForceDistribution\n");
    return false;
  }
  TiXmlHandle forceDistributionHandle(handle.FirstChild("ContactForceDistribution"));

  // Weights
  element = forceDistributionHandle.FirstChild("Weights").Element();
  if (!element) {
    printf("Could not find ContactForceDistribution:Weights\n");
    return false;
  }
  TiXmlHandle weightsHandle(handle.FirstChild("ContactForceDistribution").FirstChild("Weights"));

  // Virtual force weights
  element = weightsHandle.FirstChild("Force").Element();
  if (!element) {
    printf("Could not find ContactForceDistribution:Weights:Force!\n");
    return false;
  }
  if (element->QueryDoubleAttribute("heading", &virtualForceWeights_(0))!=TIXML_SUCCESS) {
    printf("Could not find ContactForceDistribution:Weights:Force:heading!\n");
    return false;
  }
  if (element->QueryDoubleAttribute("lateral", &virtualForceWeights_(1))!=TIXML_SUCCESS) {
    printf("Could not find ContactForceDistribution:Weights:Force:lateral!\n");
    return false;
  }
  if (element->QueryDoubleAttribute("vertical", &virtualForceWeights_(2))!=TIXML_SUCCESS) {
    printf("Could not find ContactForceDistribution:Weights:Force:vertical!\n");
    return false;
  }

  // Virtual torque weights
  element = weightsHandle.FirstChild("Torque").Element();
  if (!element) {
    printf("Could not find ContactForceDistribution:Weights:Torque!\n");
    return false;
  }
  if (element->QueryDoubleAttribute("roll", &virtualForceWeights_(3))!=TIXML_SUCCESS) {
    printf("Could not find ContactForceDistribution:Weights:Torque:roll!\n");
    return false;
  }
  if (element->QueryDoubleAttribute("pitch", &virtualForceWeights_(4))!=TIXML_SUCCESS) {
    printf("Could not find ContactForceDistribution:Weights:Torque:pitch!\n");
    return false;
  }
  if (element->QueryDoubleAttribute("yaw", &virtualForceWeights_(5))!=TIXML_SUCCESS) {
    printf("Could not find ContactForceDistribution:Weights:Torque:yaw!\n");
    return false;
  }

  // Regularizer
  element = weightsHandle.FirstChild("Regularizer").Element();
  if (!element) {
    printf("Could not find ContactForceDistribution:Weights:Regularizer!\n");
    return false;
  }
  if (element->QueryDoubleAttribute("value", &groundForceWeight_)!=TIXML_SUCCESS) {
    printf("Could not find ContactForceDistribution:Weights:Regularizer:value!\n");
    return false;
  }

  // Constraints
  element = forceDistributionHandle.FirstChild("Constraints").Element();
  if (!element) {
    printf("Could not find ContactForceDistribution:Constraints\n");
    return false;
  }
  double frictionCoefficient = 0.6;
  if (element->QueryDoubleAttribute("frictionCoefficient", &frictionCoefficient)!=TIXML_SUCCESS) {
    printf("Could not find ContactForceDistribution:Constraints:frictionCoefficient!\n");
    return false;
  }
  for (auto& legInfo : legInfos_) {
    legInfo.second.frictionCoefficient_ = frictionCoefficient;
  }
  if (element->QueryDoubleAttribute("minimalNormalForce", &minimalNormalGroundForce_)!=TIXML_SUCCESS) {
    printf("Could not find ContactForceDistribution:Constraints:minimalNormalForce!\n");
    return false;
  }

  // Load factor
  element = forceDistributionHandle.FirstChild("LoadFactor").Element();
  if (!element) {
    printf("Could not find ContactForceDistribution:Constraints\n");
    return false;
  }
  double loadFactor = 1.0;
  if (element->QueryDoubleAttribute("loadFactor", &loadFactor)!=TIXML_SUCCESS) {
    printf("Could not find ContactForceDistribution:LoadFactor:loadFactor!\n");
    return false;
  }
  for (auto& legInfo : legInfos_) {
    legInfo.first->setDesiredLoadFactor(loadFactor);
  }


  isParametersLoaded_ = true;
  return true;
}

const LegGroup* ContactForceDistribution::getLegs() const {
  return legs_.get();
}

} /* namespace loco */
