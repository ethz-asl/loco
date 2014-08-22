/*
 * MissionControlDemo.cpp
 *
 *  Created on: Aug 22, 2014
 *      Author: gech
 */

#include "loco/mission_control/MissionControlDemo.hpp"

namespace loco {

MissionControlDemo::MissionControlDemo(robotModel::RobotModel* robotModel):MissionControlJoystick(robotModel) {

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
  return true;
}

} /* namespace loco */
