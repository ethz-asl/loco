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
  Position positionWorldToDesiredBaseInWorldFrame(0.0, 0.0, 0.42);
  RotationQuaternion orientationControlToDesiredBase;
  torso_->getDesiredState().setPositionControlToBaseInControlFrame(positionWorldToDesiredBaseInWorldFrame);
  torso_->getDesiredState().setOrientationControlToBase(orientationControlToDesiredBase);
  return true;
}

bool TorsoControlJump::loadParameters(const TiXmlHandle& handle) {
  return true;
}

} /* namespace loco */
