/*
 * TorsoStateBase.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: gech
 */

#include "loco/common/TorsoStateBase.hpp"

namespace loco {

TorsoStateBase::TorsoStateBase() :
  positionWorldToBaseInWorldFrame_(),
  positionControlToBaseInControlFrame_(),
  orientationWorldToBase_(),
  orientationWorldToControl_(),
  orientationControlToBase_(),
  linearVelocityBaseInBaseFrame_(),
  angularVelocityBaseInBaseFrame_()
{

}

TorsoStateBase::~TorsoStateBase() {

}

const loco::Position& TorsoStateBase::getPositionWorldToBaseInWorldFrame() const {
  return positionWorldToBaseInWorldFrame_;
}

const RotationQuaternion& TorsoStateBase::getOrientationWorldToBase() const {
  return orientationWorldToBase_;
}

void TorsoStateBase::setOrientationWorldToBase(const RotationQuaternion& orientation) {
  orientationWorldToBase_ = orientation;
}

const loco::LinearVelocity& TorsoStateBase::getLinearVelocityBaseInBaseFrame() const {
  return linearVelocityBaseInBaseFrame_;
}

void TorsoStateBase::setLinearVelocityBaseInBaseFrame(const LinearVelocity& linearVelocity) {
  linearVelocityBaseInBaseFrame_ = linearVelocity;
}

const loco::LocalAngularVelocity& TorsoStateBase::getAngularVelocityBaseInBaseFrame() const {
  return angularVelocityBaseInBaseFrame_;
}

void TorsoStateBase::setAngularVelocityBaseInBaseFrame(const loco::LocalAngularVelocity& angularVelocity)  {
  angularVelocityBaseInBaseFrame_ = angularVelocity;
}

void TorsoStateBase::setOrientationWorldToControl(const RotationQuaternion& orientation) {
  orientationWorldToControl_ = orientation;
}

const RotationQuaternion& TorsoStateBase::getOrientationWorldToControl() const {
  return orientationWorldToControl_;
}

const RotationQuaternion&  TorsoStateBase::getOrientationControlToBase() const {
  return orientationControlToBase_;
}
void TorsoStateBase::setOrientationControlToBase(const RotationQuaternion& orientation) {
  orientationControlToBase_ = orientation;
}

void TorsoStateBase::setPositionWorldToBaseInWorldFrame(
    const Position& position) {
  positionWorldToBaseInWorldFrame_ = position;
}

const Position& TorsoStateBase::getPositionControlToBaseInControlFrame() const {
  return positionControlToBaseInControlFrame_;
}

void TorsoStateBase::setPositionControlToBaseInControlFrame(
    const Position& positionControlToBaseInControlFrame) {
  positionControlToBaseInControlFrame_ = positionControlToBaseInControlFrame;
}

const Position& TorsoStateBase::getPositionWorldToControlInWorldFrame() const {
  return positionWorldToControlInWorldFrame_;
}

void TorsoStateBase::setPositionWorldToControlInWorldFrame(
    const Position& positionWorldToControlInWorldFrame) {
  positionWorldToControlInWorldFrame_ = positionWorldToControlInWorldFrame;
}

} /* namespace loco */


