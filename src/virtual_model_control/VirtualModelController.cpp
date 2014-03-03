/*
 * VirtualModelController.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/virtual_model_control/VirtualModelController.hpp"
#include "Logger.hpp"
#include "Rotations.hpp"

using namespace std;
using namespace robotModel;
using namespace robotUtils;
using namespace Eigen;

namespace loco {

VirtualModelController::VirtualModelController(RobotModel* robotModel,
                                               ContactForceDistribution& contactForceDistribution)
    : contactForceDistribution_(contactForceDistribution),
      robotModel_(robotModel)
{
  isParametersLoaded_ = false;
  positionError_ = VectorP::Zero();
  orientationError_ = AngleAxisd::Identity();
  linearVelocityError_ = VectorP::Zero();
  angularVelocityError_ = VectorO::Zero();
  virtualForce_ = Vector3d::Zero();
  virtualTorque_ = Vector3d::Zero();

  desJointModes_.setConstant(AM_Velocity);
  desJointPositions_.setZero();
  desJointVelocities_.setZero();
  desJointTorques_.setZero();
}

VirtualModelController::~VirtualModelController()
{

}

bool VirtualModelController::addToLogger()
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
  feedforwardGainTranslation_ << 25.0, 0.0, 0.0;
  proportionalGainRotation_ << 400.0, 200.0, 10.0; // 400.0, 200.0, 0.0;
  derivativeGainRotation_ << 6.0, 9.0, 5.0; // 6.0, 9.0, 0.0;
  feedforwardGainRotation_ << 0.0, 0.0, 0.0;

  isParametersLoaded_ = true;

  return true;
}

bool VirtualModelController::computeTorques(
    const robotModel::VectorP& desiredPosition,
    const Eigen::Quaterniond& desiredOrientation,
    const robotModel::VectorP& desiredLinearVelocity,
    const robotModel::VectorO& desiredAngularVelocity)
{
  if (!isParametersLoaded()) return false;

  computePoseError(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
  computeVirtualForce(robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA() * desiredLinearVelocity);
  computeVirtualTorque(robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA() * desiredLinearVelocity);
  computeJointTorques();
  return true;
}

ContactForceDistributionBase& VirtualModelController::getContactForceDistribution() const
{
  return contactForceDistribution_;
}

void VirtualModelController::printDebugInformation()
{
  IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
  std::string sep = "\n----------------------------------------\n";
  std::cout.precision(3);

  Quaterniond actualOrientation = Quaterniond(robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA().transpose());
  Vector3d actualOrientationYawPitchRoll = Rotations::quaternionToYawPitchRoll(actualOrientation);
//  Vector3d errorYawRollPitch = Rotations::angleAxisToYawPitchRoll(orientationErrorVector_);
  Vector3d netForce, netTorque, netForceError, netTorqueError;
  contactForceDistribution_.getNetForceAndTorqueOnBase(netForce, netTorque);
  netForceError = virtualForce_ - netForce;
  netTorqueError = virtualTorque_ - netTorque;

  isParametersLoaded();
  cout << "Position error" << positionError_.format(CommaInitFmt) << endl;
  cout << "Orientation (yaw, pitch, roll)" << actualOrientationYawPitchRoll.format(CommaInitFmt) << endl;
  cout << "Orientation error in angle-axis: angle " << orientationError_.angle() << ", axis" << orientationError_.axis().format(CommaInitFmt) << endl;
  cout << "Linear velocity error" << linearVelocityError_.format(CommaInitFmt) << endl;
  cout << "Angular velocity error" << angularVelocityError_.format(CommaInitFmt)<< endl;
  cout << "Desired virtual force" << virtualForce_.format(CommaInitFmt) << endl;
  cout << "Desired virtual torque" << virtualTorque_.format(CommaInitFmt) << endl;
  cout << "Net force error" << netForceError.format(CommaInitFmt) << endl;
  cout << "Net torque error" << netTorqueError.format(CommaInitFmt) << sep;
}

bool VirtualModelController::computePoseError(
    const robotModel::VectorP& desiredPosition,
    const Eigen::Quaterniond& desiredOrientation,
    const robotModel::VectorP& desiredLinearVelocity,
    const robotModel::VectorO& desiredAngularVelocity)
{
  MatrixA transformationFromWorldToBase = robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA();

  // Errors are defined as (q_desired - q_actual).
  positionError_ = desiredPosition - robotModel_->kin().getJacobianT(JT_World2Base_CSw)->getPos();
  positionError_ = transformationFromWorldToBase * positionError_;

  // Use A.transpose() because A is alias/passive rotation but we want alibi/active rotation.
  Quaterniond actualOrientation = Quaterniond(robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA().transpose());
  Quaterniond orientationError = actualOrientation.conjugate() * desiredOrientation;
  orientationError_ = AngleAxisd(orientationError);

  linearVelocityError_ = desiredLinearVelocity - robotModel_->kin().getJacobianT(JT_World2Base_CSw)->getVel();
  linearVelocityError_ = transformationFromWorldToBase * linearVelocityError_;

  angularVelocityError_ = desiredAngularVelocity - robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getOmega();
  angularVelocityError_ = transformationFromWorldToBase * angularVelocityError_;

  return true;
}

bool VirtualModelController::computeVirtualForce(const robotModel::VectorP& desiredLinearVelocity)
{
  // Transforming gravity compensation into body frame
  Vector3d verticalDirection = robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA() * Vector3d::UnitZ();
  Vector3d gravityCompensation = robotModel_->params().mTotal_ * robotModel_->params().gravity_ * verticalDirection;

  Vector3d feedforwardTerm = Vector3d::Zero();
  feedforwardTerm.x() += desiredLinearVelocity.x();
  feedforwardTerm.y() += desiredLinearVelocity.y();

  virtualForce_ = proportionalGainTranslation_.array() * positionError_.array() // Coefficient-wise multiplication.
                       + derivativeGainTranslation_.array()   * linearVelocityError_.array()
                       + feedforwardGainTranslation_.array()  * feedforwardTerm.array()
                       + gravityCompensation.array(); // This is slightly different formulation then in C. Gehring, 2013.

  return true;
}

bool VirtualModelController::computeVirtualTorque(const robotModel::VectorO& desiredAngularVelocity)
{
  Vector3d feedforwardTerm = Vector3d::Zero();
  feedforwardTerm.z() = desiredAngularVelocity.z();

  virtualTorque_ = proportionalGainRotation_.array() * orientationError_.axis().array() * orientationError_.angle()
                       + derivativeGainRotation_.array()   * angularVelocityError_.array()
                       + feedforwardGainRotation_.array()  * feedforwardTerm.array();

  return true;
}

bool VirtualModelController::computeJointTorques()
{
  const int nDofPerLeg = 3; // TODO move to robot commons
  const int nDofPerContactPoint = 3; // TODO move to robot commons

  contactForceDistribution_.computeForceDistribution(virtualForce_, virtualTorque_); // TODO add if to check if parameters set.

  for(const auto& leg : Legs())
  {
    VectorCF contactForce = VectorCF::Zero();
    int legIndexInStackedVector = nDofPerLeg * legIndeces[leg];
    bool legInTorqueMode = contactForceDistribution_.getForceForLeg(leg, contactForce);

    if (legInTorqueMode)
    {
      VectorActLeg jointTorques;
      computeJointTorquesForLeg(leg, legIndexInStackedVector, contactForce, jointTorques);
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

void VirtualModelController::packDesiredJointSetpoints(robotModel::VectorActM& jointActuationModesToPack,
                                           robotModel::VectorAct& jointPositionsToPack,
                                           robotModel::VectorAct& jointVelocitiesToPack,
                                           robotModel::VectorAct& jointTorquesToPack) const
{
  jointActuationModesToPack = desJointModes_;
  jointPositionsToPack = desJointPositions_;
  jointVelocitiesToPack = desJointVelocities_;
  jointTorquesToPack = desJointTorques_;
}

bool VirtualModelController::computeJointTorquesForLeg(
    const Legs& leg,
    const int& legIndexInStackedVector,
    const robotModel::VectorCF& contactForce,
    robotModel::VectorActLeg& jointTorques)
{
  MatrixJ jacobian = robotModel_->kin().getJacobianTByLeg_Base2Foot_CSmb(legIndeces[leg])
      ->getJ().block<3, 3>(0, RM_NQB + legIndexInStackedVector);
  // TODO replace with "->getJ().block<nDofPerContactPoint, nDofPerLeg>(0, RM_NQB + legIndexInStackedVector);"
  jointTorques = jacobian.transpose() * contactForce;
}

bool VirtualModelController::isParametersLoaded() const
{
  if (isParametersLoaded_) return true;

  cout << "Virtual model control parameters are not loaded." << endl; // TODO use warning output
  return false;
}

} /* namespace loco */
