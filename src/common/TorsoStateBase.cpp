/*****************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Christian Gehring, Péter Fankhauser, C. Dario Bellicoso, Stelian Coros
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
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


