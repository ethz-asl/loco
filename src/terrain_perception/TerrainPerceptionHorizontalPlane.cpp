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
      gHeight += leg->getWorldToFootPositionInWorldFrame().z();
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
}


} /* namespace loco */

