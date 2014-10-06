/*
 * TorsoStateDesired.hpp
 *
 *  Created on: Mar 4, 2014
 *      Author: gech
 */

#ifndef LOCO_TORSOSTATEDESIRED_HPP_
#define LOCO_TORSOSTATEDESIRED_HPP_

#include "loco/common/TorsoStateBase.hpp"

namespace loco {

//! Desired state of the torso
class TorsoStateDesired: public TorsoStateBase {
 public:
  TorsoStateDesired();
  virtual ~TorsoStateDesired();

  void setLinearVelocityBaseInControlFrame(const LinearVelocity& linearVelocity);
  const LinearVelocity& getLinearVelocityBaseInControlFrame() const;

  void setAngularVelocityBaseInControlFrame(const LocalAngularVelocity& angularVelocity);
  const LocalAngularVelocity& getAngularVelocityBaseInControlFrame() const;



  const Position& getPositionWorldToBaseInWorldFrame() const = delete;
  void setPositionWorldToBaseInWorldFrame(const Position& position) = delete;

  const LinearVelocity& getLinearVelocityBaseInBaseFrame() const = delete;
  void setLinearVelocityBaseInBaseFrame(const LinearVelocity& linearVelocity) = delete;

  const LocalAngularVelocity& getAngularVelocityBaseInBaseFrame() const = delete;
  void setAngularVelocityBaseInBaseFrame(const loco::LocalAngularVelocity& angularVelocity) = delete;

  const RotationQuaternion& getOrientationWorldToBase() const = delete;
  void setOrientationWorldToBase(const RotationQuaternion& orientation) = delete;

 protected:

  LinearVelocity linearVelocityBaseInControlFrame_;
  LocalAngularVelocity angularVelocityBaseInControlFrame_;
};

} /* namespace loco */

#endif /* LOCO_TORSOSTATEDESIRED_HPP_ */
