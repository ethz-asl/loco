/*
 * StateBase.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_STATEBASE_HPP_
#define LOCO_STATEBASE_HPP_


#include "loco/common/TypeDefs.hpp"

namespace loco {

class TorsoBase {
 public:
  typedef loco::Twist Twist;
  typedef loco::Pose Pose;
 public:
  TorsoBase();
  virtual ~TorsoBase();

  virtual double getStridePhase() = 0;
  virtual void setStridePhase(double stridePhase) = 0;

  virtual double getHeadingSpeedInBaseFrame() = 0;
  virtual double getTurningSpeedInBaseFrame() = 0;
  virtual double getLateralSpeedInBaseFrame() = 0;

  virtual double getDesiredHeadingSpeedInBaseFrame() = 0;
  virtual double getDesiredTurningSpeedInBaseFrame() = 0;
  virtual double getDesiredLateralSpeedInBaseFrame() = 0;

  virtual void setMeasuredTwistInBaseFrame(const Twist& twist) = 0;
  virtual void setDesiredTwistInBaseFrame(const Twist& twist) = 0;

  virtual void setMeasuredPoseInWorldFrame(const Pose& pose) = 0;
  virtual void setDesiredPoseInWorldFrame(const Pose& pose) = 0;

  virtual const Twist& getMeasuredTwistInBaseFrame() = 0;
  virtual const Twist& getDesiredTwistInBaseFrame() = 0;

  virtual const Pose& getMeasuredPoseInWorldFrame() = 0;
  virtual const Pose& getDesiredPoseInWorldFrame() = 0;



};

} /* namespace loco */

#endif /* STATEBASE_HPP_ */
