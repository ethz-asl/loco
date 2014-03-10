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

bool VirtualModelController::addToLogger()
{
  robotUtils::addEigenMatrixToLog(virtualForce_.toImplementation(), "VMC_desired_force", "N", true);
  robotUtils::addEigenMatrixToLog(virtualTorque_.toImplementation(), "VMC_desired_torque", "Nm", true);
  robotUtils::updateLogger();
  return true;
}

bool VirtualModelController::compute()
{
  if (!isParametersLoaded()) return false;

  computeError();
  computeGravityCompensation();
  computeVirtualForce();
  computeVirtualTorque();
  if (!contactForceDistribution_->computeForceDistribution(virtualForce_, virtualTorque_)) {
    return false;
  }
//  cout << *this << endl;
  return true;
}

bool VirtualModelController::computeError()
{
  // Errors are defined as (q_desired - q_actual).

  const Position positionErrorInWorldFrame_ = torso_->getDesiredState().getWorldToBasePositionInWorldFrame()
      - torso_->getMeasuredState().getWorldToBasePositionInWorldFrame();
  positionErrorInBaseFrame_ = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame().rotate(positionErrorInWorldFrame_);

  orientationError_ = torso_->getDesiredState().getWorldToBaseOrientationInWorldFrame().boxMinus(
      torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame());

  linearVelocityError_ = torso_->getDesiredState().getBaseLinearVelocityInBaseFrame() - torso_->getMeasuredState().getBaseLinearVelocityInBaseFrame();

  angularVelocityError_ = torso_->getDesiredState().getBaseAngularVelocityInBaseFrame() - torso_->getMeasuredState().getBaseAngularVelocityInBaseFrame();

  return true;
}

bool VirtualModelController::computeGravityCompensation()
{
  const LinearAcceleration gravitationalAccelerationInWorldFrame = torso_->getProperties().getGravity();
  const LinearAcceleration gravitationalAccelerationInBaseFrame = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame().rotate(gravitationalAccelerationInWorldFrame);

  gravityCompensationForce_ = Force(-torso_->getProperties().getMass() * gravitationalAccelerationInBaseFrame);

  for (const auto& leg : *legs_)
  {
    gravityCompensationForce_ += Force(-leg->getProperties().getMass() * gravitationalAccelerationInBaseFrame);
  }

  gravityCompensationTorque_ = Torque(
      torso_->getProperties().getBaseToCenterOfMassPositionInBaseFrame().cross(-torso_->getProperties().getMass() * gravitationalAccelerationInBaseFrame));
  for (const auto& leg : *legs_)
  {
    gravityCompensationTorque_ += Torque(
      leg->getProperties().getBaseToCenterOfMassPositionInBaseFrame().cross(-leg->getProperties().getMass() * gravitationalAccelerationInBaseFrame));
  }

  return true;
}

bool VirtualModelController::computeVirtualForce()
{
  Vector3d feedforwardTerm = Vector3d::Zero();
  feedforwardTerm.x() += torso_->getDesiredState().getBaseLinearVelocityInBaseFrame().x();
  feedforwardTerm.y() += torso_->getDesiredState().getBaseLinearVelocityInBaseFrame().y();

  virtualForce_ = Force(proportionalGainTranslation_.cwiseProduct(positionErrorInBaseFrame_.toImplementation())
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

std::ostream& operator << (std::ostream& out, const VirtualModelController& motionController)
{
  IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
  std::cout.precision(3);

  kindr::rotations::eigen_impl::EulerAnglesYprPD errorYawRollPitch(motionController.orientationError_);
  Force netForce, netForceError;
  Torque netTorque, netTorqueError;
  motionController.contactForceDistribution_->getNetForceAndTorqueOnBase(netForce, netTorque);
  netForceError = motionController.virtualForce_ - netForce;
  netTorqueError = motionController.virtualTorque_ - netTorque;

  motionController.isParametersLoaded();
  out << "Position error" << motionController.positionErrorInBaseFrame_.toImplementation().format(CommaInitFmt) << endl;
  out << "Orientation error" << motionController.orientationError_.format(CommaInitFmt) << endl;
  out << "Linear velocity error" << motionController.linearVelocityError_.toImplementation().format(CommaInitFmt) << endl;
  out << "Angular velocity error" << motionController.angularVelocityError_.toImplementation().format(CommaInitFmt) << endl;
  out << "Gravity compensation force" << motionController.gravityCompensationForce_.toImplementation().format(CommaInitFmt) << endl;
  out << "Gravity compensation torque" << motionController.gravityCompensationTorque_.toImplementation().format(CommaInitFmt) << endl;
  out << "Desired virtual force" << motionController.virtualForce_.toImplementation().format(CommaInitFmt) << endl;
  out << "Desired virtual torque" << motionController.virtualTorque_.toImplementation().format(CommaInitFmt) << endl;
  out << "Net force error" << netForceError.toImplementation().format(CommaInitFmt) << endl;
  out << "Net torque error" << netTorqueError.toImplementation().format(CommaInitFmt) << endl;
  return out;
}

Force VirtualModelController::getDesiredVirtualForceInBaseFrame() const {
  return virtualForce_;
}

Torque VirtualModelController::getDesiredVirtualTorqueInBaseFrame() const {
  return virtualTorque_;
}

void VirtualModelController::getDistributedVirtualForceAndTorqueInBaseFrame(Force& netForce, Torque& netTorque) const {
  contactForceDistribution_->getNetForceAndTorqueOnBase(netForce, netTorque);
}

bool VirtualModelController::loadParameters(const TiXmlHandle& handle)
{
  isParametersLoaded_ = false;
  TiXmlElement* element;

  TiXmlHandle gainsHandle(handle.FirstChild("VirtualModelController").FirstChild("Gains"));
  element = gainsHandle.Element();
  if (!element) {
    printf("Could not find VirtualModelController:Gains\n");
    return false;
  }

  // Heading
  element = gainsHandle.FirstChild("Heading").Element();
  if (element->QueryDoubleAttribute("kp", &proportionalGainTranslation_.x())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Heading:kp\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kd", &derivativeGainTranslation_.x())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Heading:kd\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kff", &feedforwardGainTranslation_.x())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Heading:kff\n");
    return false;
  }

  // Lateral
  element = gainsHandle.FirstChild("Lateral").Element();
  if (element->QueryDoubleAttribute("kp", &proportionalGainTranslation_.y())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Lateral:kp\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kd", &derivativeGainTranslation_.y())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Lateral:kd\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kff", &feedforwardGainTranslation_.y())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Lateral:kff\n");
    return false;
  }

  // Vertical
  element = gainsHandle.FirstChild("Vertical").Element();
  if (element->QueryDoubleAttribute("kp", &proportionalGainTranslation_.z())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Vertical:kp\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kd", &derivativeGainTranslation_.z())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Vertical:kd\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kff", &feedforwardGainTranslation_.z())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Vertical:kff\n");
    return false;
  }

  // Roll
  element = gainsHandle.FirstChild("Roll").Element();
  if (element->QueryDoubleAttribute("kp", &proportionalGainRotation_.x())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Roll:kp\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kd", &derivativeGainRotation_.x())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Roll:kd\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kff", &feedforwardGainRotation_.x())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Roll:kff\n");
    return false;
  }

  // Pitch
  element = gainsHandle.FirstChild("Pitch").Element();
  if (element->QueryDoubleAttribute("kp", &proportionalGainRotation_.y())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Pitch:kp\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kd", &derivativeGainRotation_.y())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Pitch:kd\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kff", &feedforwardGainRotation_.y())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Pitch:kff\n");
    return false;
  }

  // Yaw
  element = gainsHandle.FirstChild("Yaw").Element();
  if (element->QueryDoubleAttribute("kp", &proportionalGainRotation_.z())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Yaw:kp\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kd", &derivativeGainRotation_.z())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Yaw:kd\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kff", &feedforwardGainRotation_.z())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Yaw:kff\n");
    return false;
  }

  isParametersLoaded_ = true;
  return true;
}


} /* namespace loco */
