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
    gaitSwitcher_->transitToGait("WalkingTrot");
  }

  return true;
}

} /* namespace loco */
