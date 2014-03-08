/*
 * VirtualModelController.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/motion_control/VirtualModelController.hpp"
#include "Logger.hpp"
#include "kindr/rotations/eigen/EulerAnglesZyx.hpp"

using namespace std;
using namespace Eigen;

namespace loco {

VirtualModelController::VirtualModelController(std::shared_ptr<LegGroup> legs, std::shared_ptr<TorsoBase> torso,
                                               std::shared_ptr<ContactForceDistributionBase> contactForceDistribution)
    : MotionControllerBase(legs, torso),
      contactForceDistribution_(contactForceDistribution)
{

}

VirtualModelController::~VirtualModelController()
{

}

bool VirtualModelController::loadParameters()
{
  // TODO Replace this with proper parameters loading (XML)

  // walk
  proportionalGainTranslation_ << 500.0, 640.0, 600.0;
  derivativeGainTranslation_ << 150.0, 100.0, 120.0;
  feedforwardGainTranslation_ << 25.0, 0.0, 0.0;
  proportionalGainRotation_ << 400.0, 200.0, 10.0; // 400.0, 200.0, 0.0;
  derivativeGainRotation_ << 6.0, 9.0, 5.0; // 6.0, 9.0, 0.0;
  feedforwardGainRotation_ << 0.0, 0.0, 0.0;

  return MotionControllerBase::loadParameters();
}

bool VirtualModelController::addToLogger()
{
  robotUtils::addEigenMatrixToLog(virtualForce_.toImplementation(), "VMC_desired_force", "N", true);
  robotUtils::addEigenMatrixToLog(virtualTorque_.toImplementation(), "VMC_desired_torque", "Nm", true);
  robotUtils::updateLogger();
  return MotionControllerBase::addToLogger();
}

bool VirtualModelController::compute()
{
  if (!isParametersLoaded()) return false;

  computeError();
  computeGravityCompensation();
  computeVirtualForce();
  computeVirtualTorque();
  contactForceDistribution_->computeForceDistribution(virtualForce_, virtualTorque_);
  printDebugInformation();
  return true;
}

bool VirtualModelController::computeError()
{
  // Errors are defined as (q_desired - q_actual).

  positionError_ = torso_->getDesiredState().getWorldToBasePositionInWorldFrame()
      - torso_->getMeasuredState().getWorldToBasePositionInWorldFrame();
  positionError_ = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame().rotate(positionError_);

  orientationError_ = torso_->getDesiredState().getWorldToBaseOrientationInWorldFrame().boxMinus(
      torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame());

  linearVelocityError_ = torso_->getDesiredState().getBaseLinearVelocityInBaseFrame() - torso_->getMeasuredState().getBaseLinearVelocityInBaseFrame();

  angularVelocityError_ = torso_->getDesiredState().getBaseAngularVelocityInBaseFrame() - torso_->getMeasuredState().getBaseAngularVelocityInBaseFrame();

  return true;
}

bool VirtualModelController::computeGravityCompensation()
{
  // Transforming gravity compensation into body frame
  // TODO Make this with full calculations with all bodies.
  // TODO Make this getting mass and gravity from common module.
  // TODO Make acceleration.
  Vector3d verticalDirection = Vector3d::UnitZ();
  verticalDirection = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame().rotate(verticalDirection);
  gravityCompensationForce_ = Force(25.0 * 9.81 * verticalDirection);

  gravityCompensationTorque_.setZero();
  return true;
}

bool VirtualModelController::computeVirtualForce()
{
  Vector3d feedforwardTerm = Vector3d::Zero();
  feedforwardTerm.x() += torso_->getDesiredState().getBaseLinearVelocityInBaseFrame().x();
  feedforwardTerm.y() += torso_->getDesiredState().getBaseLinearVelocityInBaseFrame().y();

  virtualForce_ = Force(proportionalGainTranslation_.cwiseProduct(positionError_.toImplementation())
                       + derivativeGainTranslation_.cwiseProduct(linearVelocityError_.toImplementation())
                       + feedforwardGainTranslation_.cwiseProduct(feedforwardTerm)
                       + gravityCompensationForce_.toImplementation());

  return true;
}

bool VirtualModelController::computeVirtualTorque()
{
  Vector3d feedforwardTerm = Vector3d::Zero();
  feedforwardTerm.z() += torso_->getDesiredState().getBaseAngularVelocityInBaseFrame().z();

  virtualTorque_ = Torque(proportionalGainRotation_.cwiseProduct(orientationError_)
                       + derivativeGainRotation_.cwiseProduct(angularVelocityError_.toImplementation())
                       + feedforwardGainRotation_.cwiseProduct(feedforwardTerm)
                       + gravityCompensationTorque_.toImplementation());

  return true;
}

bool VirtualModelController::isParametersLoaded() const
{
  if (isParametersLoaded_) return true;

  cout << "Virtual model control parameters are not loaded." << endl; // TODO use warning output
  return false;
}

void VirtualModelController::printDebugInformation()
{
  IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
  std::string sep = "\n----------------------------------------\n";
  std::cout.precision(3);

  kindr::rotations::eigen_impl::EulerAnglesYprPD errorYawRollPitch(orientationError_);
  Force netForce, netForceError;
  Torque netTorque, netTorqueError;
  contactForceDistribution_->getNetForceAndTorqueOnBase(netForce, netTorque);
  netForceError = virtualForce_ - netForce;
  netTorqueError = virtualTorque_ - netTorque;

  isParametersLoaded();
  cout << "Position error" << positionError_.toImplementation().format(CommaInitFmt) << endl;
  cout << "Orientation error" << errorYawRollPitch.toImplementation().format(CommaInitFmt) << endl;
  cout << "Linear velocity error" << linearVelocityError_.toImplementation().format(CommaInitFmt) << endl;
  cout << "Angular velocity error" << angularVelocityError_.toImplementation().format(CommaInitFmt) << endl;
  cout << "Desired virtual force" << virtualForce_.toImplementation().format(CommaInitFmt) << endl;
  cout << "Desired virtual torque" << virtualTorque_.toImplementation().format(CommaInitFmt) << endl;
  cout << "Net force error" << netForceError.toImplementation().format(CommaInitFmt) << endl;
  cout << "Net torque error" << netTorqueError.toImplementation().format(CommaInitFmt) << sep;
}

} /* namespace loco */
