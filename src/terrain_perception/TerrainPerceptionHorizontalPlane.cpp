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
 * TerrainPerceptionHorizontalPlane.cpp
 *
 *  Created on: Apr 15, 2014
 *      Author: gech
 */

#include "loco/terrain_perception/TerrainPerceptionHorizontalPlane.hpp"

namespace loco {

TerrainPerceptionHorizontalPlane::TerrainPerceptionHorizontalPlane(TerrainModelHorizontalPlane* terrainModel, LegGroup* legs, TorsoBase* torso) :
  TerrainPerceptionBase(),
  terrainModel_(terrainModel),
  legs_(legs),
  torso_(torso)
{

}

TerrainPerceptionHorizontalPlane::~TerrainPerceptionHorizontalPlane() {

}

bool TerrainPerceptionHorizontalPlane::initialize(double dt) {
  if(!advance(dt)) {
    return false;
  }
  return true;
}

bool TerrainPerceptionHorizontalPlane::advance(double dt) {
  int groundedLimbCount = 0;
  double gHeight = 0.0;


  for (auto leg : *legs_) {
    if (leg->isAndShouldBeGrounded()){
      groundedLimbCount++;
      gHeight += leg->getPositionWorldToFootInWorldFrame().z();
    }
  }

  if (groundedLimbCount > 0) {
    terrainModel_->setHeight(gHeight / groundedLimbCount);
  }

  updateControlFrameOrigin();
  updateControlFrameAttitude();

  return true;
}


void TerrainPerceptionHorizontalPlane::updateControlFrameOrigin() {
  //--- Position of the control frame is equal to the position of the world frame.
  torso_->getMeasuredState().setPositionWorldToControlInWorldFrame(Position::Zero());
  //---
}


void TerrainPerceptionHorizontalPlane::updateControlFrameAttitude() {
  //--- hack set control frame equal to heading frame
  RotationQuaternion orientationWorldToControl;
  EulerAnglesZyx orientationWorldToHeadingEulerZyx = EulerAnglesZyx(torso_->getMeasuredState().getOrientationWorldToBase()).getUnique();
  orientationWorldToHeadingEulerZyx.setPitch(0.0);
  orientationWorldToHeadingEulerZyx.setRoll(0.0);
  orientationWorldToControl = RotationQuaternion(orientationWorldToHeadingEulerZyx.getUnique());

  torso_->getMeasuredState().setOrientationWorldToControl(orientationWorldToControl);
  torso_->getMeasuredState().setPositionControlToBaseInControlFrame(orientationWorldToControl.rotate(torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame() - torso_->getMeasuredState().getPositionWorldToControlInWorldFrame()));

  RotationQuaternion orientationWorldToBase = torso_->getMeasuredState().getOrientationWorldToBase();
  torso_->getMeasuredState().setOrientationControlToBase(orientationWorldToBase*orientationWorldToControl.inverted());




//  EulerAnglesZyx worldToControlEuler = EulerAnglesZyx(orientationWorldToControl).getUnique();
//  std::cout << "*******" << std::endl;
//  std::cout << "orientation world to control: " << std::endl << worldToControlEuler.roll() << " "
//                                                             << worldToControlEuler.pitch() << " "
//                                                             << worldToControlEuler.yaw() << std::endl;
//  std::cout << "*******" << std::endl << std::endl;
//  EulerAnglesZyx worldToBaseEuler = EulerAnglesZyx(torso_->getMeasuredState().getOrientationWorldToBase()).getUnique();
//  std::cout << "*******" << std::endl;
//  std::cout << "orientation world to base: " << std::endl << worldToBaseEuler.roll() << " "
//                                                             << worldToBaseEuler.pitch() << " "
//                                                             << worldToBaseEuler.yaw() << std::endl;
//  std::cout << "*******" << std::endl << std::endl;

}


} /* namespace loco */

