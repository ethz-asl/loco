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
 * LimbCoordinatorJump.cpp
 *
 *  Created on: Oct 6, 2014
 *      Author: gech
 */

#include "loco/limb_coordinator/LimbCoordinatorJump.hpp"
#include "starlethModel/RobotModel_common.hpp"

namespace loco {

LimbCoordinatorJump::LimbCoordinatorJump(LegGroup* legs, TorsoBase* torso) : LimbCoordinatorBase(),
    legs_(legs),
    torso_(torso)
{

}

LimbCoordinatorJump::~LimbCoordinatorJump()
{
}

bool LimbCoordinatorJump::initialize(double dt) {
  return true;
}

bool LimbCoordinatorJump::advance(double dt) {

  StateSwitcher* stateSwitcher;
  LegBase::JointControlModes desiredJointControlModes;

   for (auto leg : *legs_) {

     stateSwitcher = leg->getStateSwitcher();
     stateSwitcher->setState(StateSwitcher::States::StanceNormal);
     if (leg->isGrounded()) {
       leg->setIsSupportLeg(true);
     }

     //--- Set control mode
     if (leg->isSupportLeg()) {
       desiredJointControlModes.setConstant(robotModel::AM_Torque);
     }
     else {
       desiredJointControlModes.setConstant(robotModel::AM_Position);
     }
     leg->setDesiredJointControlModes(desiredJointControlModes);
     //---
   }
  return true;
}

bool LimbCoordinatorJump::loadParameters(const TiXmlHandle& handle) {
  return true;
}

GaitPatternBase* LimbCoordinatorJump::getGaitPattern() {
  return nullptr;
}

} /* namespace loco */
