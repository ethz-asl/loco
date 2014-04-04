/*
 * MissionControlJoystick.hpp
 *
 *  Created on: Mar 7, 2014
 *      Author: gech
 */

#ifndef LOCO_MISSIONCONTROLJOYSTICK_HPP_
#define LOCO_MISSIONCONTROLJOYSTICK_HPP_

#include "loco/mission_control/MissionControlBase.hpp"
#include "RobotModel.hpp"
#include "loco/temp_helpers/FilteredVariable.hpp"

namespace loco {

class MissionControlJoystick: public MissionControlBase {
 public:
  MissionControlJoystick(robotModel::RobotModel* robotModel);
  virtual ~MissionControlJoystick();

  virtual bool initialize(double dt);
  virtual void advance(double dt);
  virtual const Twist& getDesiredBaseTwistInBaseFrame() const;

  virtual bool loadParameters(const TiXmlHandle& handle);
 protected:
  robotModel::RobotModel* robotModel_;
  Twist baseTwistInBaseFrame_;
  Twist maximumBaseTwistInBaseFrame_;
  //! filtered speeds [sagittal; coronal; turning]
  FilteredDouble filteredVelocities_[3];

};

} /* namespace loco */

#endif /* LOCO_MISSIONCONTROLJOYSTICK_HPP_ */
