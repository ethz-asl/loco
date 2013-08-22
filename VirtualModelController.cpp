/*
 * VirtualModelController.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "VirtualModelController.hpp"
#include "Logger.hpp"
#include "Rotations.hpp"

using namespace std;
using namespace robotModel;
using namespace robotController;
using namespace robotUtils;
using namespace Eigen;

namespace robotController {

VirtualModelController::VirtualModelController(RobotModel* robotModel) : ControllerBase(robotModel)
{
  contactForceDistribution_ = new ContactForceDistribution(robotModel);
  isParametersLoaded_ = false;
  positionError_ = Vector3d::Zero(); // TODO set type right
  orientationErrorVector_ = Vector3d::Zero();
  linearVelocityError_ = Vector3d::Zero();  // TODO set type right
  angularVelocityError_ = Vector3d::Zero();  // TODO set type right
  virtualForce_ = Vector3d::Zero();
  virtualTorque_ = Vector3d::Zero();
}

VirtualModelController::~VirtualModelController()
{
  delete contactForceDistribution_;
}

bool VirtualModelController::initialize()
{
  addEigenMatrixToLog(virtualForce_, "VMC_desired_force", "N", true);
  addEigenMatrixToLog(virtualTorque_, "VMC_desired_torque", "Nm", true);
  updateLogger();
  return true;
}

bool VirtualModelController::loadParameters()
{
  // TODO Replace this with proper parameters loading (XML)
  proportionalGainTranslation_ << 500.0, 640.0, 600.0;
  derivativeGainTranslation_ << 150.0, 100.0, 120.0;
  feedforwardGainTranslation_ << 25.0, 0.0, 1.0;
  proportionalGainRotation_ << 400.0, 200.0, 0.0;
  derivativeGainRotation_ << 6.0, 9.0, 0.0;
  feedforwardGainRotation_ << 0.0, 0.0, 0.0;

  // For debugging
//  proportionalGainTranslation_ << 1.0, 1.0, 1.0;
//  derivativeGainTranslation_ << 1.0, 1.0, 1.0;
//  feedforwardGainTranslation_ << 0.0, 0.0, 0.0;
//  proportionalGainRotation_ << 1.0, 1.0, 1.0;
//  derivativeGainRotation_ << 1.0, 1.0, 1.0;
//  feedforwardGainRotation_ << 0.0, 0.0, 0.0;

  contactForceDistribution_->loadParameters();

  isParametersLoaded_ = true;

  return true;
}

bool VirtualModelController::computeTorques(
    const robotModel::VectorP& desiredPosition,
    const Eigen::Quaterniond& desiredOrientation,
    const robotModel::VectorP& desiredLinearVelocity,
    const robotModel::VectorO& desiredAngularVelocity)
{
  if (!areParametersLoaded()) return false;

  computePoseError(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
  computeVirtualForce(desiredLinearVelocity);
  computeVirtualTorque(desiredAngularVelocity);
  computeJointTorques();
  return true;
}

void VirtualModelController::printDebugInformation() const
{
  IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
  std::string sep = "\n----------------------------------------\n";
  std::cout.precision(3);

  Vector3d rollPitchYawVector;
  Rotations::rotVecToRpy(orientationErrorVector_, rollPitchYawVector);

  areParametersLoaded();
  cout << "Position error: " << positionError_.format(CommaInitFmt) << endl;
  cout << "Orientation error (roll, pitch, yaw)" << rollPitchYawVector.format(CommaInitFmt) << endl;
  cout << "Orientation error (rotation vector)" << orientationErrorVector_.format(CommaInitFmt) << endl;
  cout << "Linear velocity error" << linearVelocityError_.format(CommaInitFmt) << endl;
  cout << "Angular velocity error" << angularVelocityError_.format(CommaInitFmt)<< endl;
  cout << "Desired virtual force" << virtualForce_.format(CommaInitFmt) << endl;
  cout << "Desired virtual torque" << virtualTorque_.format(CommaInitFmt) << sep;
}

bool VirtualModelController::computePoseError(
    const robotModel::VectorP& desiredPosition,
    const Eigen::Quaterniond& desiredOrientation,
    const robotModel::VectorP& desiredLinearVelocity,
    const robotModel::VectorO& desiredAngularVelocity)
{
  MatrixA transformationFromWorldToBase = robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA();

  positionError_ = desiredPosition - robotModel_->kin().getJacobianT(JT_World2Base_CSw)->getPos();
  positionError_ = transformationFromWorldToBase * positionError_;

  Quaterniond actualOrientation = Quaterniond(robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA());
  Quaterniond orientationError = desiredOrientation.conjugate() * actualOrientation;
  orientationErrorVector_ = AngleAxisd(orientationError).angle() * AngleAxisd(orientationError).axis();
  orientationErrorVector_ = transformationFromWorldToBase * orientationErrorVector_;

  linearVelocityError_ = desiredLinearVelocity - robotModel_->kin().getJacobianT(JT_World2Base_CSw)->getVel();
  linearVelocityError_ = transformationFromWorldToBase * linearVelocityError_;

  angularVelocityError_ = desiredAngularVelocity - robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getOmega();
  angularVelocityError_ = transformationFromWorldToBase * angularVelocityError_;

  return true;
}

bool VirtualModelController::computeVirtualForce(const robotModel::VectorP& desiredLinearVelocity)
{
  Vector3d feedforwardTerm = Vector3d::Zero();
  feedforwardTerm.x() = desiredLinearVelocity.x();
  feedforwardTerm.y() = desiredLinearVelocity.y();
  feedforwardTerm.z() = robotModel_->params().mTotal_ * robotModel_->params().gravity_;

  virtualForce_ = proportionalGainTranslation_.array() * positionError_.array() // Coefficient-wise multiplication.
                       + derivativeGainTranslation_.array()   * linearVelocityError_.array()
                       + feedforwardGainTranslation_.array()  * feedforwardTerm.array();

  // TODO figure this out which frame
  //desiredVirtualForce_ = robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA() * desiredVirtualForce_;

  return true;
}

bool VirtualModelController::computeVirtualTorque(const robotModel::VectorO& desiredAngularVelocity)
{
  Vector3d feedforwardTerm = Vector3d::Zero();
  feedforwardTerm.z() = desiredAngularVelocity.z();

  virtualTorque_ = proportionalGainRotation_.array() * orientationErrorVector_.array()
                       + derivativeGainRotation_.array()   * angularVelocityError_.array()
                       + feedforwardGainRotation_.array()  * feedforwardTerm.array();

  // TODO figure this out which frame
  //desiredVirtualTorque_ = robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA() * desiredVirtualTorque_;

  return true;
}

bool VirtualModelController::computeJointTorques()
{
  const int nDofPerLeg = 3; // TODO move to robot commons
  const int nDofPerContactPoint = 3; // TODO move to robot commons
  contactForceDistribution_->computeForceDistribution(virtualForce_, virtualTorque_);

  VectorCF contactForce = VectorCF::Zero();
  contactForceDistribution_->getForceForLeg(ContactForceDistribution::LEFT_FRONT, contactForce);

  MatrixJ jacobian = robotModel_->kin().getJacobianTByLeg_Base2Foot_CSw(0)->getJ().block<nDofPerContactPoint, nDofPerLeg>(0, RM_NQB);

  VectorXd jointTorques = jacobian.transpose() * contactForce;
  desJointTorques_.segment<nDofPerLeg>(qjLF_HAA) = jointTorques;
  desJointModes_.segment<nDofPerLeg>(qjLF_HAA) = VectorActMLeg::Constant(AM_Torque);



//  VectorCF contactForce = VectorCF::Zero();
//  bool inStance = contactForceDistribution_->getForceForLeg(ContactForceDistribution::LEFT_FRONT, contactForce);
//  if (inStance)
//    cout << "Leg LEFT_FRONT: " << endl << contactForce.transpose() << endl;
//
//  inStance = contactForceDistribution_->getForceForLeg(ContactForceDistribution::RIGHT_FRONT, contactForce);
//  if (inStance)
//    cout << "Leg RIGHT_FRONT: " << endl << contactForce.transpose() << endl;
//
//  inStance = contactForceDistribution_->getForceForLeg(ContactForceDistribution::LEFT_HIND, contactForce);
//  if (inStance)
//    cout << "Leg LEFT_HIND: " << endl << contactForce.transpose() << endl;
//
//  inStance = contactForceDistribution_->getForceForLeg(ContactForceDistribution::RIGHT_HIND, contactForce);
//  if (inStance)
//    cout << "Leg RIGHT_HIND: " << endl << contactForce.transpose() << endl;

//  // Get foot jacobian
//  MatrixJ jacobian = robotModel_->kin().getJacobianT(JT_Base2Foot_CSw)->getJ();
//  // TODO: Change to JT_CoG2Foot_CSw?
//
//  //! Check if base has reached touchdown height
//  if (zeroForceAtTouchdownHeight && basePosition.z() >= z_0 - delta_z)
//  {
//          virtualSpringForce.setZero();
//  }
//
//  // Compute joint torques
//  VectorQ generalizedForces = jacobian.transpose() * virtualSpringForce;
//  desJointTorques_(qjHFE) = generalizedForces(qHFE);
//  desJointTorques_(qjKFE) = generalizedForces(qKFE);


  return true;
}

bool VirtualModelController::areParametersLoaded() const
{
  if (isParametersLoaded_) return true;

  cout << "Virtual model control parameters are not loaded." << endl; // TODO use warning output
  return false;
}

} /* namespace robotController */
