/*
 * StateDynamicGait.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_STATEDYNAMICGAIT_HPP_
#define LOCO_STATEDYNAMICGAIT_HPP_


#include "loco/common/TorsoBase.hpp"
#include "kindr/poses/PoseDiffEigen.hpp"
#include "kindr/poses/PoseEigen.hpp"
#include <Eigen/Core>

namespace loco {


class TorsoStarlETH: public TorsoBase {
 public:
  typedef kindr::positions::eigen_impl::LinearVelocityD LinearVelocity;
  typedef kindr::rotations::eigen_impl::LocalAngularVelocityPD LocalAngularVelocity;

 public:
  TorsoStarlETH();
  virtual ~TorsoStarlETH();

  virtual double getStridePhase();
  virtual void setStridePhase(double stridePhase);

  virtual double getHeadingSpeedInBaseFrame();
  virtual double getTurningSpeedInBaseFrame();
  virtual double getLateralSpeedInBaseFrame();

  virtual double getDesiredHeadingSpeedInBaseFrame();
  virtual double getDesiredTurningSpeedInBaseFrame();
  virtual double getDesiredLateralSpeedInBaseFrame();


  virtual void setMeasuredTwistInBaseFrame(const Twist& twist);
  virtual void setDesiredTwistInBaseFrame(const Twist& twist);

  virtual void setMeasuredPoseInWorldFrame(const Pose& pose);
  virtual void setDesiredPoseInWorldFrame(const Pose& pose);

  virtual const Twist& getMeasuredTwistInBaseFrame();
  virtual const Twist& getDesiredTwistInBaseFrame();

  virtual const Pose& getMeasuredPoseInWorldFrame();
  virtual const Pose& getDesiredPoseInWorldFrame();

protected:
  double stridePhase_;
  Pose desiredPoseInWorldFrame_;
  Pose measuredPoseInWorldFrame_;
  Twist desiredTwistInBaseFrame_;
  Twist measuredTwistInBaseFrame_;





};

} /* namespace loco */

#endif /* LOCO_STATEDYNAMICGAIT_HPP_ */
