/*
 * MissionControlStaticGait.hpp
 *
 *  Created on: Aug 22, 2014
 *      Author: gech
 */

#ifndef LOCO_MissionControlStaticGait_HPP_
#define LOCO_MissionControlStaticGait_HPP_

#include "loco/mission_control/MissionControlBase.hpp"
#include "loco/mission_control/MissionControlSpeedFilter.hpp"
#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"
#include "starlethModel/RobotModel.hpp"

namespace loco {

class MissionControlStaticGait : public MissionControlBase {
 public:
  MissionControlStaticGait(robotModel::RobotModel* robotModel, LocomotionControllerDynamicGait* locomotionController);
  virtual ~MissionControlStaticGait();

  virtual bool initialize(double dt);
  virtual bool advance(double dt);
  virtual bool loadParameters(const TiXmlHandle& handle);
  virtual const Twist& getDesiredBaseTwistInHeadingFrame() const;
 private:
  double interpolateJoystickAxis(double value, double minValue, double maxValue);
 private:
  robotModel::RobotModel* robotModel_;
  bool isExternallyVelocityControlled_;
  MissionControlSpeedFilter speedFilter_;
  LocomotionControllerDynamicGait* locomotionController_;

  bool useRosService_;

};

} /* namespace loco */

#endif /* MissionControlStaticGait_HPP_ */
