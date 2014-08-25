/*
 * MissionControlDemo.cpp
 *
 *  Created on: Aug 22, 2014
 *      Author: gech
 */

#include "loco/mission_control/MissionControlDemo.hpp"

namespace loco {

MissionControlDemo::MissionControlDemo(robotModel::RobotModel* robotModel,   GaitSwitcherDynamicGaitDefault* gaitSwitcher):
    MissionControlJoystick(robotModel),
    gaitSwitcher_(gaitSwitcher)
{

}

MissionControlDemo::~MissionControlDemo() {

}

bool MissionControlDemo::initialize(double dt) {
  if(!MissionControlJoystick::initialize(dt)) {
    return false;
  }
  return true;
}

bool MissionControlDemo::advance(double dt) {
  if(!MissionControlJoystick::advance(dt)) {
    return false;
  }

  robotUtils::Joystick* joyStick = robotModel_->sensors().getJoystick();

  if (joyStick->getButtonOneClick(1)) {
    gaitSwitcher_->transitToGait("Stand");
  }

  if (joyStick->getButtonOneClick(2)) {
    gaitSwitcher_->transitToGait("StaticLateralWalk");
  }

  if (joyStick->getButtonOneClick(3)) {
    gaitSwitcher_->transitToGait("WalkingTrot");
  }

  if (joyStick->getButtonOneClick(4)) {
    gaitSwitcher_->transitToGait("Pronk");
  }
  return true;
}

} /* namespace loco */
