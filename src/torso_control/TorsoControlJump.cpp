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
 * TorsoControlJump.cpp
 *
 *  Created on: Oct 7, 2014
 *      Author: gech
 */

#include "loco/torso_control/TorsoControlJump.hpp"

namespace loco {

TorsoControlJump::TorsoControlJump(LegGroup* legs, TorsoBase* torso,  loco::TerrainModelBase* terrain) :
TorsoControlBase(),
legs_(legs),
torso_(torso),
terrain_(terrain)
{

}

TorsoControlJump::~TorsoControlJump()
{

}

bool TorsoControlJump::initialize(double dt) {
  return true;
}

bool TorsoControlJump::advance(double dt) {
  for (auto leg : *legs_) {
    if (leg->isSupportLeg()) {
      // this leg supports the torso, hence, it can be used to control the torso
    }
  }
  Position positionWorldToDesiredBaseInControlFrame(0.0, 0.0, 0.42);
  RotationQuaternion orientationControlToDesiredBase;
  torso_->getDesiredState().setPositionControlToBaseInControlFrame(positionWorldToDesiredBaseInControlFrame);
  torso_->getDesiredState().setOrientationControlToBase(orientationControlToDesiredBase);
  return true;
}

bool TorsoControlJump::loadParameters(const TiXmlHandle& handle) {
  return true;
}

bool TorsoControlJump::setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t) {
  return true;
}
void TorsoControlJump::setDesiredPositionOffsetInWorldFrame(const Position& positionOffsetInWorldFrame) {

}
void TorsoControlJump::setDesiredOrientationOffset(const RotationQuaternion& orientationOffset) {

}

} /* namespace loco */
