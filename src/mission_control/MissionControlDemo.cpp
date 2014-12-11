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
 * @file    MissionControlDemo.cpp
 * @author  Christian Gehring
 * @date    Aug 22, 2014
 * @version 1.0
 * @ingroup loco
 */
#include "loco/mission_control/MissionControlDemo.hpp"
#include "loco/mission_control/MissionControlSpeedFilter.hpp"

namespace loco {

MissionControlDemo::MissionControlDemo(robotModel::RobotModel* robotModel,   GaitSwitcherDynamicGaitDefault* gaitSwitcher):
    missionControlDynamicGait_(robotModel, gaitSwitcher),
    robotModel_(robotModel),
    gaitSwitcher_(gaitSwitcher),
    isExternallyVelocityControlled_(false),
    isJoystickActive_(true)
{

}

MissionControlDemo::~MissionControlDemo() {

}

bool MissionControlDemo::initialize(double dt) {
  if(!missionControlDynamicGait_.initialize(dt)) {
    std::cout << "Could not initialize MissionControlDynamicGait!\n";
    return false;
  }
  isExternallyVelocityControlled_ = false;

  return true;
}

bool MissionControlDemo::advance(double dt) {

  //-
  if (isJoystickActive_) {
    const double minStrideDuration_ = 0.6;
    const double maxStrideDuration_ = 0.9;
    const double stepStrideDuration_ = 0.05;

    Position desiredPositionOffsetInControlFrame;
    RotationQuaternion desiredOrientationOffset;
    LinearVelocity desiredLinearVelocityBaseInControlFrame;
    LocalAngularVelocity desiredAngularVelocityBaseInControlFrame;

    robotUtils::Joystick* joyStick = robotModel_->sensors().getJoystick();


    MissionControlSpeedFilter& missionController = gaitSwitcher_->getLocomotionController()->getMissionController();

    if (gaitSwitcher_->getLocomotionController()->getGaitName() == "Stand") {
      desiredPositionOffsetInControlFrame.z() = interpolateJoystickAxis(joyStick->getVertical(), missionController.getMinimalPositionOffsetInWorldFrame().z(), missionController.getMaximalPositionOffsetInWorldFrame().z());
//      RotationQuaternion desiredPositionOffsetInControlFrame(EulerAnglesZyx(0.0, M_PI/4, 0.0));

  //    printf("sag: %lf \t cor: %lf  \t yaw: %lf\n", joyStick->getSagittal(), joyStick->getCoronal(), joyStick->getYaw());
      EulerAnglesZyx desEulerAnglesZyx;
      desEulerAnglesZyx.setRoll(interpolateJoystickAxis(-joyStick->getCoronal(), missionController.getMinimalDesiredOrientationOffset().getUnique().roll(), missionController.getMaximalDesiredOrientationOffset().getUnique().roll()));
      desEulerAnglesZyx.setPitch(interpolateJoystickAxis(joyStick->getSagittal(), missionController.getMinimalDesiredOrientationOffset().getUnique().pitch(), missionController.getMaximalDesiredOrientationOffset().getUnique().pitch()));
      desEulerAnglesZyx.setYaw(interpolateJoystickAxis(joyStick->getYaw(), missionController.getMinimalDesiredOrientationOffset().getUnique().yaw(), missionController.getMaximalDesiredOrientationOffset().getUnique().yaw()));
      desiredOrientationOffset(desEulerAnglesZyx.getUnique());

    } else if (gaitSwitcher_->getLocomotionController()->getGaitName() == "WalkingTrot") {


      // xbox cross down
      if (joyStick->getButtonOneClick(15)) { // 15
        missionControlDynamicGait_.increaseStrideDuration();
        printf("Increased stride duration!\n");
      }

      // xbox cross up
      if (joyStick->getButtonOneClick(14)) { // 14
        missionControlDynamicGait_.decreaseStrideDuration();
        printf("Decreased stride duration!\n");
      }


      desiredLinearVelocityBaseInControlFrame.x() = joyStick->getSagittal();
      desiredLinearVelocityBaseInControlFrame.y() = joyStick->getCoronal();
      desiredLinearVelocityBaseInControlFrame.z() = 0.0;
      desiredAngularVelocityBaseInControlFrame.z() = joyStick->getYaw();

      missionController.setUnfilteredDesiredOrientationOffset(RotationQuaternion());
      missionController.setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(Position());

    } else if (gaitSwitcher_->getLocomotionController()->getGaitName() == "StaticLateralWalk") {

      desiredLinearVelocityBaseInControlFrame.x() = joyStick->getSagittal();
      desiredLinearVelocityBaseInControlFrame.y() = joyStick->getCoronal();
      desiredLinearVelocityBaseInControlFrame.z() = 0.0;
      desiredAngularVelocityBaseInControlFrame.z() = joyStick->getYaw();
      missionController.setUnfilteredDesiredOrientationOffset(RotationQuaternion());
      missionController.setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(Position());

    }


    if (joyStick->getButtonOneClick(1)) {
      missionControlDynamicGait_.switchToStand();
    }

    if (joyStick->getButtonOneClick(2)) {
      //missionControlDynamicGait_.switchToStaticLateralWalk();
    }

    if (joyStick->getButtonOneClick(3)) {
      missionControlDynamicGait_.switchToWalkingTrot();
    }

    if (joyStick->getButtonOneClick(4)) {
      isExternallyVelocityControlled_ = !isExternallyVelocityControlled_;
      if (isExternallyVelocityControlled_) {
        printf("Velocity is controlled from extern.\n");
      } else {
        printf("Velocity is controlled by joystick.\n");
      }
    }

    if (isExternallyVelocityControlled_) {

      desiredLinearVelocityBaseInControlFrame.x() = robotModel_->sensors().getDesRobotVelocity()->getDesSagittalVelocity();
      desiredLinearVelocityBaseInControlFrame.y() = robotModel_->sensors().getDesRobotVelocity()->getDesCoronalVelocity();
      desiredAngularVelocityBaseInControlFrame.z() = robotModel_->sensors().getDesRobotVelocity()->getDesTurningRate();
    }


    missionControlDynamicGait_.setDesiredPositionOffsetInControlFrame(desiredPositionOffsetInControlFrame);
    missionControlDynamicGait_.setDesiredOrientationOffset(desiredOrientationOffset);
    missionControlDynamicGait_.setDesiredLinearVelocityBaseInControlFrame(desiredLinearVelocityBaseInControlFrame);
    missionControlDynamicGait_.setDesiredAngularVelocityBaseInControlFrame(desiredAngularVelocityBaseInControlFrame);

  }
  //---

  if (!missionControlDynamicGait_.advance(dt)){
    return false;
  }
  return true;
}


bool MissionControlDemo::loadParameters(const TiXmlHandle& handle) {
  return true;
}

const Twist& MissionControlDemo::getDesiredBaseTwistInHeadingFrame() const {
  return gaitSwitcher_->getLocomotionController()->getDesiredBaseTwistInHeadingFrame();
}

double MissionControlDemo::interpolateJoystickAxis(double value, double minValue, double maxValue) {
  return 0.5*(maxValue-minValue)*(value+1.0) + minValue;
}

MissionControlDynamicGait* MissionControlDemo::getMissionControlDynamicGait() {
  return &missionControlDynamicGait_;
}

void MissionControlDemo::setIsJoystickActive(bool isActive) {
  isJoystickActive_ = isActive;
}

} /* namespace loco */
