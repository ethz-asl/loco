/*
 * MissionControlDemo.hpp
 *
 *  Created on: Aug 22, 2014
 *      Author: gech
 */

#ifndef LOCO_MISSIONCONTROLDEMO_HPP_
#define LOCO_MISSIONCONTROLDEMO_HPP_

#include "loco/mission_control/MissionControlJoystick.hpp"
#include "loco/gait_switcher/GaitSwitcherDynamicGaitDefault.hpp"
#include "RobotModel.hpp"

namespace loco {

class MissionControlDemo : public MissionControlJoystick {
 public:
  MissionControlDemo(robotModel::RobotModel* robotModel, GaitSwitcherDynamicGaitDefault* gaitSwitcher);
  virtual ~MissionControlDemo();

  virtual bool initialize(double dt);
  virtual bool advance(double dt);
 private:
  GaitSwitcherDynamicGaitDefault* gaitSwitcher_;
};

} /* namespace loco */

#endif /* MISSIONCONTROLDEMO_HPP_ */
