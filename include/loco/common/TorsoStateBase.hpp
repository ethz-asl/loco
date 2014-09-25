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

  const Position& getPositionWorldToBaseInWorldFrame() const;
  void setPositionWorldToBaseInWorldFrame(const Position& position);

  const Position& getPositionControlToBaseInControlFrame() const;
  void setPositionControlToBaseInControlFrame(
      const Position& positionControlToBaseInControlFrame);

  const LinearVelocity& getLinearVelocityBaseInBaseFrame() const;
  void setLinearVelocityBaseInBaseFrame(const LinearVelocity& linearVelocity);

  const LocalAngularVelocity& getAngularVelocityBaseInBaseFrame() const;
  void setAngularVelocityBaseInBaseFrame(const loco::LocalAngularVelocity& angularVelocity);

  const RotationQuaternion& getOrientationWorldToBase() const;
  void setOrientationWorldToBase(const RotationQuaternion& orientation);

  // good:
  const RotationQuaternion& getOrientationWorldToControl() const;
  void setOrientationWorldToControl(const RotationQuaternion& orientation);

  const RotationQuaternion&  getOrientationControlToBase() const;
  void setOrientationControlToBase(const RotationQuaternion& orientation);
  const Position& getPositionWorldToControlInWorldFrame() const;
  void setPositionWorldToControlInWorldFrame(
      const Position& positionWorldToControlInWorldFrame);

protected:
  Position positionWorldToBaseInWorldFrame_;
  Position positionWorldToControlInWorldFrame_;
  Position positionControlToBaseInControlFrame_;

  RotationQuaternion orientationWorldToBase_;
  RotationQuaternion orientationWorldToControl_;
  RotationQuaternion orientationControlToBase_;


  LinearVelocity linearVelocityBaseInBaseFrame_;
  LocalAngularVelocity angularVelocityBaseInBaseFrame_;



};

} /* namespace loco */

#endif /* LOCO_TORSOSTATEBASE_HPP_ */
