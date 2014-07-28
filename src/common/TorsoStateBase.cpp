/*
 * TorsoStateBase.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: gech
 */

#include "loco/common/TorsoStateBase.hpp"

namespace loco {

TorsoStateBase::TorsoStateBase() {

}

TorsoStateBase::~TorsoStateBase() {

}


void TorsoStateBase::setBaseTwistInBaseFrame(const Twist& twist) {
  linearVelocityBaseInBaseFrame_ = twist.getTranslationalVelocity();
  angularVelocityBaseInBaseFrame_ = twist.getRotationalVelocity();
  twistBaseInBaseFrame_ = twist;
}

const loco::Position& TorsoStateBase::getWorldToBasePositionInWorldFrame() const {
  return positionWorldToBaseInWorldFrame_;
}

const RotationQuaternion& TorsoStateBase::getWorldToBaseOrientationInWorldFrame() const {
  return orientationWorldToBaseInWorldFrame_;
}

const loco::Twist& TorsoStateBase::getBaseTwistInBaseFrame() const {
  return twistBaseInBaseFrame_;
}



double TorsoStateBase::getHeadingSpeedInBaseFrame() const {
  return twistBaseInBaseFrame_.getTranslationalVelocity().x();
}

double TorsoStateBase::getTurningSpeedInBaseFrame() const {
  return twistBaseInBaseFrame_.getRotationalVelocity().z();
}

double TorsoStateBase::getLateralSpeedInBaseFrame() const {
  return twistBaseInBaseFrame_.getTranslationalVelocity().y();
}

const loco::LinearVelocity& TorsoStateBase::getBaseLinearVelocityInBaseFrame() const {
  return linearVelocityBaseInBaseFrame_;
}

const loco::LocalAngularVelocity& TorsoStateBase::getBaseAngularVelocityInBaseFrame() const {
  return angularVelocityBaseInBaseFrame_;
}

const loco::Pose& TorsoStateBase::getWorldToBasePoseInWorldFrame() {
    return poseBaseToWorldInWorldFrame_;
}

void TorsoStateBase::setWorldToBasePoseInWorldFrame(const Pose& pose) {
  poseBaseToWorldInWorldFrame_ = pose;
  positionWorldToBaseInWorldFrame_ = pose.getPosition();
  orientationWorldToBaseInWorldFrame_ = pose.getRotation();
}


const RotationQuaternion& TorsoStateBase::getWorldToHeadingOrientation() const {
  return orientationWorldToHeading_;
}
void TorsoStateBase::setWorldToHeadingOrientation(const RotationQuaternion& orientation) {
  orientationWorldToHeading_ = orientation;
}

const RotationQuaternion&  TorsoStateBase::getHeadingToBaseOrientation() const {
  return orientationHeadingToBase_;
}
void TorsoStateBase::setHeadingToBaseOrientation(const RotationQuaternion& orientation) {
  orientationHeadingToBase_ = orientation;
}

std::ostream& operator << (std::ostream& out, const TorsoStateBase& state) {
  out << "Position (world): " << state.positionWorldToBaseInWorldFrame_ << std::endl;
  out << "Orientation (Euler Zyx): " << EulerAnglesZyx(state.orientationWorldToBaseInWorldFrame_).getUnique() << std::endl;
  out << "Linear velocity (base): " << state.linearVelocityBaseInBaseFrame_ << std::endl;
  out << "Angular velocity (base): " << state.angularVelocityBaseInBaseFrame_ << std::endl;
  return out;
}

} /* namespace loco */
