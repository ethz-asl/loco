/*
 * MissionControlDemo.hpp
 *
 *  Created on: Aug 22, 2014
 *      Author: gech
 */

#ifndef LOCO_MISSIONCONTROLDEMO_HPP_
#define LOCO_MISSIONCONTROLDEMO_HPP_

#include "loco/mission_control/MissionControlBase.hpp"
#include "loco/gait_switcher/GaitSwitcherDynamicGaitDefault.hpp"
#include "RobotModel.hpp"

namespace loco {

class MissionControlDemo : public MissionControlBase {
 public:
  MissionControlDemo(robotModel::RobotModel* robotModel, GaitSwitcherDynamicGaitDefault* gaitSwitcher);
  virtual ~MissionControlDemo();

  virtual bool initialize(double dt);
  virtual bool advance(double dt);
  virtual bool loadParameters(const TiXmlHandle& handle);
  virtual const Twist& getDesiredBaseTwistInHeadingFrame() const;
 private:
  double interpolateJoystickAxis(double value, double minValue, double maxValue);
 private:
  robotModel::RobotModel* robotModel_;
  GaitSwitcherDynamicGaitDefault* gaitSwitcher_;
  bool isExternallyVelocityControlled_;
};

} /* namespace loco */

#endif /* MISSIONCONTROLDEMO_HPP_ */
