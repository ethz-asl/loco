/*
 * Copyright (c) 2014, Christian Gehring, C. Dario Bellicoso, Peter Fankhauser
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Christian Gehring, C. Dario Bellicoso, Peter Fankhauser
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
/*!
 * @file    MissionControlDynamicGait.hpp
 * @author  Christian Gehring
 * @date    Nov 2014
 * @version 1.0
 * @ingroup loco
 */
#ifndef LOCO_MISSIONCONTROLDYNAMICGAIT_HPP_
#define LOCO_MISSIONCONTROLDYNAMICGAIT_HPP_

#include "loco/mission_control/MissionControlBase.hpp"
#include "loco/gait_switcher/GaitSwitcherDynamicGaitDefault.hpp"
#include "starlethModel/RobotModel.hpp"

namespace loco {

class MissionControlDynamicGait: public MissionControlBase
{
 public:
  MissionControlDynamicGait(robotModel::RobotModel* robotModel, GaitSwitcherDynamicGaitDefault* gaitSwitcher);
  virtual ~MissionControlDynamicGait();
  virtual const Twist& getDesiredBaseTwistInHeadingFrame() const;
  virtual bool initialize(double dt);
  virtual bool advance(double dt);
  virtual bool loadParameters(const TiXmlHandle& handle);
  bool switchToWalkingTrot();
  bool switchToStand();
  bool switchToStaticLateralWalk();
  void setDesiredPositionOffsetInControlFrame(const Position& desiredPositionOffsetInControlFrame);
  void setDesiredOrientationOffset(const RotationQuaternion& desiredOrientationOffset);
  void setDesiredLinearVelocityBaseInControlFrame(const LinearVelocity& desiredLinearVelocityBaseInControlFrame);
  void setDesiredAngularVelocityBaseInControlFrame(const LocalAngularVelocity& desiredAngularVelocityBaseInControlFrame);
  bool setStrideDuration(double strideDuration);
  bool increaseStrideDuration();
  bool decreaseStrideDuration();
 private:
  robotModel::RobotModel* robotModel_;
  GaitSwitcherDynamicGaitDefault* gaitSwitcher_;
  Position desiredPositionOffsetInControlFrame_;
  RotationQuaternion desiredOrientationOffset_;
  LinearVelocity desiredLinearVelocityBaseInControlFrame_;
  LocalAngularVelocity desiredAngularVelocityBaseInControlFrame_;
  double minStrideDuration_;
  double maxStrideDuration_;
  double stepStrideDuration_;
};

} /* namespace loco */

#endif /* MISSIONCONTROLDYNAMICGAIT_HPP_ */
