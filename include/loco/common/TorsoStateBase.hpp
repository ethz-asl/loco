/*****************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Christian Gehring, Péter Fankhauser, C. Dario Bellicoso, Stelian Coros
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
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
