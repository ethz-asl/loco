/*
 * TorsoStateBase.hpp
 *
 *  Created on: Mar 4, 2014
 *      Author: gech
 */

#ifndef LOCO_TORSOSTATEBASE_HPP_
#define LOCO_TORSOSTATEBASE_HPP_

#include "loco/common/TypeDefs.hpp"

namespace loco {

class TorsoStateBase {
 public:
  TorsoStateBase();
  virtual ~TorsoStateBase();

  double getHeadingSpeedInBaseFrame() const;
  double getTurningSpeedInBaseFrame() const;
  double getLateralSpeedInBaseFrame() const;


  const LinearVelocity& getBaseLinearVelocityInBaseFrame() const;
  const LocalAngularVelocity& getBaseAngularVelocityInBaseFrame() const;
  const Twist& getBaseTwistInBaseFrame() const;

  const Position& getWorldToBasePositionInWorldFrame() const;
  const RotationQuaternion& getWorldToBaseOrientationInWorldFrame() const;
  const Pose& getWorldToBasePoseInWorldFrame();

  void setBaseTwistInBaseFrame(const Twist& twist);
  void setWorldToBasePoseInWorldFrame(const Pose& pose);

  const RotationQuaternion&  getWorldToHeadingOrientation() const;
  void setWorldToHeadingOrientation(const RotationQuaternion& orientation);

  const RotationQuaternion&  getHeadingToBaseOrientation() const;
  void setHeadingToBaseOrientation(const RotationQuaternion& orientation);

 protected:
  Position positionWorldToBaseInWorldFrame_;
  RotationQuaternion orientationWorldToBaseInWorldFrame_;
  RotationQuaternion orientationWorldToHeading_;
  RotationQuaternion orientationHeadingToBase_;
  Pose poseBaseToWorldInWorldFrame_;

  LinearVelocity linearVelocityBaseInBaseFrame_;
  LocalAngularVelocity angularVelocityBaseInBaseFrame_;
  Twist twistBaseInBaseFrame_;



};

} /* namespace loco */

#endif /* LOCO_TORSOSTATEBASE_HPP_ */
