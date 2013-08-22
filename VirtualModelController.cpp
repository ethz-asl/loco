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
  proportionalGainRotation_ << 400.0, 200.0, 10.0; //< 400.0, 200.0, 0.0;
  derivativeGainRotation_ << 6.0, 9.0, 5.0; // 6.0, 9.0, 0.0;
  feedforwardGainRotation_ << 0.0, 0.0, 0.0;

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
  // TODO feedforward should be in body frame too (gravity in m*g)!
  Vector3d feedforwardTerm = Vector3d::Zero();
  feedforwardTerm.x() = desiredLinearVelocity.x();
  feedforwardTerm.y() = desiredLinearVelocity.y();
  feedforwardTerm.z() = robotModel_->params().mTotal_ * robotModel_->params().gravity_;

  virtualForce_ = proportionalGainTranslation_.array() * positionError_.array() // Coefficient-wise multiplication.
                       + derivativeGainTranslation_.array()   * linearVelocityError_.array()
                       + feedforwardGainTranslation_.array()  * feedforwardTerm.array();

  return true;
}

bool VirtualModelController::computeVirtualTorque(const robotModel::VectorO& desiredAngularVelocity)
{
  Vector3d feedforwardTerm = Vector3d::Zero();
  feedforwardTerm.z() = desiredAngularVelocity.z();

  virtualTorque_ = proportionalGainRotation_.array() * orientationErrorVector_.array()
                       + derivativeGainRotation_.array()   * angularVelocityError_.array()
                       + feedforwardGainRotation_.array()  * feedforwardTerm.array();

  return true;
}

bool VirtualModelController::computeJointTorques()
{
  const int nDofPerLeg = 3; // TODO move to robot commons
  const int nDofPerContactPoint = 3; // TODO move to robot commons

  contactForceDistribution_->computeForceDistribution(virtualForce_, virtualTorque_);

  for (ContactForceDistribution::LegNumber legNumber = // TODO ugly!!!
      ContactForceDistribution::LEFT_FRONT;
      legNumber <= ContactForceDistribution::RIGHT_HIND; legNumber =
          ContactForceDistribution::LegNumber(legNumber + 1))
  {
    VectorCF contactForce = VectorCF::Zero();
    int legIndexInStackedVector = nDofPerLeg * (int)legNumber;
    bool legInTorqueMode = contactForceDistribution_->getForceForLeg(legNumber, contactForce);

    if (legInTorqueMode)
    {
      VectorActLeg jointTorques;
      computeJointTorquesForLeg(legNumber, legIndexInStackedVector, contactForce, jointTorques);
      desJointTorques_.segment<nDofPerLeg>(legIndexInStackedVector) = jointTorques;
      desJointModes_.segment<nDofPerLeg>(legIndexInStackedVector) = VectorActMLeg::Constant(AM_Torque);
    }
    else
    {
      desJointModes_.segment<nDofPerLeg>(legIndexInStackedVector) = VectorActMLeg::Constant(AM_Velocity);
    }
  }

  return true;
}

bool VirtualModelController::computeJointTorquesForLeg(
    const ContactForceDistribution::LegNumber& legNumber,
    const int& legIndexInStackedVector,
    const robotModel::VectorCF& contactForce,
    robotModel::VectorActLeg& jointTorques)
{
  MatrixJ jacobian = robotModel_->kin().getJacobianTByLeg_Base2Foot_CSmb(legNumber)
      ->getJ().block<3, 3>(0, RM_NQB + legIndexInStackedVector);
  // TODO replace with "->getJ().block<nDofPerContactPoint, nDofPerLeg>(0, RM_NQB + legIndexInStackedVector);"
  jointTorques = jacobian.transpose() * contactForce;
}

bool VirtualModelController::areParametersLoaded() const
{
  if (isParametersLoaded_) return true;

  cout << "Virtual model control parameters are not loaded." << endl; // TODO use warning output
  return false;
}

} /* namespace robotController */
