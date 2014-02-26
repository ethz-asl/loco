/*
 * StateBase.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_STATEBASE_HPP_
#define LOCO_STATEBASE_HPP_

#include "kindr/poses/PoseDiffEigen.hpp"
#include "kindr/poses/PoseEigen.hpp"
namespace loco {

class TorsoBase {
 public:
  typedef kindr::poses::eigen_impl::TwistLinearVelocityLocalAngularVelocityD Twist;
  typedef kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD Pose;
 public:
  TorsoBase();
  virtual ~TorsoBase();


  virtual double getHeadingSpeedInBaseFrame() = 0;
  virtual double getTurningSpeedInBaseFrame() = 0;
  virtual double getLateralSpeedInBaseFrame() = 0;

  virtual double getDesiredHeadingSpeedInBaseFrame() = 0;
  virtual double getDesiredTurningSpeedInBaseFrame() = 0;
  virtual double getDesiredLateralSpeedInBaseFrame() = 0;

  virtual void setMeasuredTwistInBaseFrame(const Twist& twist) = 0;
  virtual void setDesiredTwistInBaseFrame(const Twist& twist) = 0;

  virtual void setMeasuredPoseInBaseFrame(const Pose& pose) = 0;
  virtual void setDesiredPoseInBaseFrame(const Pose& pose) = 0;

  virtual const Twist& getMeasuredTwistInBaseFrame() = 0;
  virtual const Twist& getDesiredTwistInBaseFrame() = 0;

  virtual const Pose& getMeasuredPoseInBaseFrame() = 0;
  virtual const Pose& getDesiredPoseInBaseFrame() = 0;

};

} /* namespace loco */

#endif /* STATEBASE_HPP_ */
