/*
 * Copyright (c) 2014, Christian Gehring, C. Dario Bellicoso, Peter Fankhauser
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Christian Gehring, C. Dario Bellicoso, Peter Fankhauser
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
/*!
 * @file    MissionControlDynamicGait.cpp
 * @author  Christian Gehring
 * @date    Nov 2014
 * @version 1.0
 * @ingroup loco
 */
#include "loco/mission_control/MissionControlDynamicGait.hpp"

namespace loco {

MissionControlDynamicGait::MissionControlDynamicGait(robotModel::RobotModel* robotModel,   GaitSwitcherDynamicGaitDefault* gaitSwitcher):
        robotModel_(robotModel),
        gaitSwitcher_(gaitSwitcher),
        desiredPositionOffsetInControlFrame_(),
        desiredOrientationOffset_(),
        desiredLinearVelocityBaseInControlFrame_(),
        desiredAngularVelocityBaseInControlFrame_(),
        minStrideDuration_(0.6),
        maxStrideDuration_(0.9),
        stepStrideDuration_(0.05)
{

}

MissionControlDynamicGait::~MissionControlDynamicGait()
{

}

const Twist& MissionControlDynamicGait::getDesiredBaseTwistInHeadingFrame() const {
  return gaitSwitcher_->getLocomotionController()->getDesiredBaseTwistInHeadingFrame();
}

bool MissionControlDynamicGait::initialize(double dt) {
  desiredPositionOffsetInControlFrame_.setZero();
  desiredOrientationOffset_.setIdentity();
  desiredLinearVelocityBaseInControlFrame_.setZero();
  desiredAngularVelocityBaseInControlFrame_.setZero();
  return true;
}

bool MissionControlDynamicGait::advance(double dt) {
  MissionControlSpeedFilter& missionController = gaitSwitcher_->getLocomotionController()->getMissionController();
  if (gaitSwitcher_->getLocomotionController()->getGaitName() == "Stand") {
    missionController.setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(desiredPositionOffsetInControlFrame_);
    missionController.setUnfilteredDesiredOrientationOffset(desiredOrientationOffset_);
    gaitSwitcher_->getLocomotionController()->setDesiredBaseTwistInHeadingFrame(Twist());
  }
  else {
    missionController.setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(Position());
    missionController.setUnfilteredDesiredOrientationOffset(RotationQuaternion());
    Twist desiredBaseTwistInHeadingFrame(desiredLinearVelocityBaseInControlFrame_, desiredAngularVelocityBaseInControlFrame_);
    gaitSwitcher_->getLocomotionController()->setDesiredBaseTwistInHeadingFrame(desiredBaseTwistInHeadingFrame);
  }

  return true;
}

bool MissionControlDynamicGait::loadParameters(const TiXmlHandle& handle) {
  return true;
}

bool MissionControlDynamicGait::switchToWalkingTrot() {
  return gaitSwitcher_->transitToGait("WalkingTrot");
}
bool MissionControlDynamicGait::switchToStand() {
  return gaitSwitcher_->transitToGait("Stand");
}
bool MissionControlDynamicGait::switchToStaticLateralWalk() {
  return gaitSwitcher_->transitToGait("StaticLateralWalk");
}

void MissionControlDynamicGait::setDesiredPositionOffsetInControlFrame(const Position& desiredPositionOffsetInControlFrame) {
  desiredPositionOffsetInControlFrame_ = desiredPositionOffsetInControlFrame;
}
void MissionControlDynamicGait::setDesiredOrientationOffset(const RotationQuaternion& desiredOrientationOffset) {
  desiredOrientationOffset_ = desiredOrientationOffset;
}
void MissionControlDynamicGait::setDesiredLinearVelocityBaseInControlFrame(const LinearVelocity& desiredLinearVelocityBaseInControlFrame) {
  desiredLinearVelocityBaseInControlFrame_ = desiredLinearVelocityBaseInControlFrame;
}
void MissionControlDynamicGait::setDesiredAngularVelocityBaseInControlFrame(const LocalAngularVelocity& desiredAngularVelocityBaseInControlFrame) {
  desiredAngularVelocityBaseInControlFrame_ = desiredAngularVelocityBaseInControlFrame;
}
bool MissionControlDynamicGait::setStrideDuration(double strideDuration) {
  if (gaitSwitcher_->getLocomotionController()->getGaitName() == "WalkingTrot") {
    strideDuration = boundToRange(strideDuration, minStrideDuration_,  maxStrideDuration_);
    gaitSwitcher_->getLocomotionController()->getGaitPattern()->setStrideDuration(strideDuration);
  }
  return true;
}

bool MissionControlDynamicGait::increaseStrideDuration() {
  if (gaitSwitcher_->getLocomotionController()->getGaitName() == "WalkingTrot") {
    double strideDuration = gaitSwitcher_->getLocomotionController()->getGaitPattern()->getStrideDuration() + stepStrideDuration_;
    strideDuration = boundToRange(strideDuration, minStrideDuration_,  maxStrideDuration_);
    gaitSwitcher_->getLocomotionController()->getGaitPattern()->setStrideDuration(strideDuration);
  }
  return true;
}

bool MissionControlDynamicGait::decreaseStrideDuration() {
  if (gaitSwitcher_->getLocomotionController()->getGaitName() == "WalkingTrot") {
    double strideDuration = gaitSwitcher_->getLocomotionController()->getGaitPattern()->getStrideDuration() - stepStrideDuration_;
    strideDuration = boundToRange(strideDuration, minStrideDuration_,  maxStrideDuration_);
    gaitSwitcher_->getLocomotionController()->getGaitPattern()->setStrideDuration(strideDuration);

  }
  return true;
}

} /* namespace loco */
