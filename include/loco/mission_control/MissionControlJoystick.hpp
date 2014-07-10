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
  const Twist& getDesiredBaseTwistInHeadingFrame() const;
  const Position& getDesiredPositionMiddleOfFeetToBaseInWorldFrame() const;
  const RotationQuaternion& getDesiredOrientationHeadingToBase() const;

  virtual bool loadParameters(const TiXmlHandle& handle);

  double interpolateJoystickAxis(double value, double minValue, double maxValue);

  friend std::ostream& operator << (std::ostream& out, const MissionControlJoystick& joystick);
 protected:
  robotModel::RobotModel* robotModel_;
  Twist baseTwistInHeadingFrame_;
  Twist maximumBaseTwistInHeadingFrame_;
  Position desiredPositionMiddleOfFeetToBaseInWorldFrame_;
  Position minimalPositionMiddleOfFeetToBaseInWorldFrame_;
  Position maximalPositionMiddleOfFeetToBaseInWorldFrame_;
  RotationQuaternion desiredOrientationHeadingToBase_;
  EulerAnglesZyx minimalOrientationHeadingToBase_;
  EulerAnglesZyx maximalOrientationHeadingToBase_;

  //! filtered speeds [sagittal; coronal; turning]
  FilteredDouble filteredVelocities_[3];




};

} /* namespace loco */

#endif /* LOCO_MISSIONCONTROLJOYSTICK_HPP_ */
