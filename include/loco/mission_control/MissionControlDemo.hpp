/*
 * MissionControlDemo.hpp
 *
 *  Created on: Aug 22, 2014
 *      Author: gech
 */

#ifndef LOCO_MISSIONCONTROLDEMO_HPP_
#define LOCO_MISSIONCONTROLDEMO_HPP_

#include "loco/mission_control/MissionControlJoystick.hpp"
#include "RobotModel.hpp"

namespace loco {

class MissionControlDemo : public MissionControlJoystick {
 public:
  MissionControlDemo(robotModel::RobotModel* robotModel);
  virtual ~MissionControlDemo();

  virtual bool initialize(double dt);
  virtual bool advance(double dt);
};

} /* namespace loco */

#endif /* MISSIONCONTROLDEMO_HPP_ */
