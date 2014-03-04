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

} /* namespace loco */
