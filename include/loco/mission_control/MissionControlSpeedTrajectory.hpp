/*
 * MissionControlZeroToConstantHeadingVelocity.hpp
 *
 *  Created on: Apr 4, 2014
 *      Author: gech
 */

#ifndef LOCO_MISSIONCONTROLZEROTOCONSTANTHEADINGVELOCITY_HPP_
#define LOCO_MISSIONCONTROLZEROTOCONSTANTHEADINGVELOCITY_HPP_

#include "loco/mission_control/MissionControlBase.hpp"
#include "loco/temp_helpers/Trajectory.hpp"

namespace loco {

class MissionControlSpeedTrajectory: public MissionControlBase {
 public:
  MissionControlSpeedTrajectory();
  virtual ~MissionControlSpeedTrajectory();

  virtual const Twist& getDesiredBaseTwistInBaseFrame() const;
  virtual bool initialize(double dt);
  virtual void advance(double dt);
  virtual bool loadParameters(const TiXmlHandle& handle);
 protected:
  double time_;
  loco::TrajectoryLinearVelocity linearVelocityTrajectory_;
  loco::TrajectoryLocalAngularVelocity localAngularVelocityTrajectory_;
  Twist currentBaseTwistInBaseFrame_;

};

} /* namespace loco */

#endif /* LOCO_MISSIONCONTROLZEROTOCONSTANTHEADINGVELOCITY_HPP_ */
