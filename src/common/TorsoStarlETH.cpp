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
 * TorsoStarlETH.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/common/TorsoStarlETH.hpp"

namespace loco {

TorsoStarlETH::TorsoStarlETH(robotModel::RobotModel* robotModel)
    : TorsoBase(),
      robotModel_(robotModel),
      properties_(robotModel),
      stridePhase_(0.0)
{

}

TorsoStarlETH::~TorsoStarlETH()
{

}

double TorsoStarlETH::getStridePhase()
{
  return stridePhase_;
}

void TorsoStarlETH::setStridePhase(double stridePhase)
{
  stridePhase_ = stridePhase;
}

TorsoStateMeasured& TorsoStarlETH::getMeasuredState()
{
  return stateMeasured_;
}

TorsoStateDesired& TorsoStarlETH::getDesiredState()
{
  return stateDesired_;
}

const TorsoStateDesired& TorsoStarlETH::getDesiredState() const {
  return stateDesired_;
}

TorsoPropertiesBase& TorsoStarlETH::getProperties()
{
  return static_cast<TorsoPropertiesBase&>(properties_);
}



bool TorsoStarlETH::initialize(double dt)
{
  if(!this->getProperties().initialize(dt)) {
    return false;
  }
  if (!this->advance(dt)) {
    return false;
  }
  return true;
}

bool TorsoStarlETH::advance(double dt)
{
  if(!getProperties().advance(dt)) {
    return false;
  }

  kindr::rotations::eigen_impl::RotationQuaternionAD rquatWorldToBaseActive(
      robotModel_->est().getActualEstimator()->getQuat());
  const RotationQuaternion  orientationWorldToBase = rquatWorldToBaseActive.getPassive();
  const Position positionWorldToBaseInWorldFrame = Position(
      robotModel_->kin()[robotModel::JT_World2Base_CSw]->getPos());
//  std::cout << "world2base position: " << robotModel_->kin()[robotModel::JT_World2Base_CSw]->getPos() << std::endl;
//  std::cout << "position: " << rquatWorldToBase.rotate(robotModel_->kin()[robotModel::JT_World2Base_CSw]->getPos()) << std::endl;

  this->getMeasuredState().setPositionWorldToBaseInWorldFrame(positionWorldToBaseInWorldFrame);
  this->getMeasuredState().setOrientationWorldToBase(orientationWorldToBase);
  const LinearVelocity linearVelocityInBaseFrame(
      orientationWorldToBase.rotate(robotModel_->kin()[robotModel::JT_World2Base_CSw]->getVel()));
  const LocalAngularVelocity localAngularVelocityInBaseFrame(
      orientationWorldToBase.rotate(robotModel_->kin()(robotModel::JR_World2Base_CSw)->getOmega()));

  this->getMeasuredState().setLinearVelocityBaseInBaseFrame(linearVelocityInBaseFrame);
  this->getMeasuredState().setAngularVelocityBaseInBaseFrame(localAngularVelocityInBaseFrame);

//  std::cout << "lin vel in base frame: " << linearVelocityInBaseFrame << std::endl;


//  EulerAnglesZyx orientationWorldToHeadingEulerZyx = EulerAnglesZyx(orientationWorldToBase).getUnique();
//  orientationWorldToHeadingEulerZyx.setPitch(0.0);
//  orientationWorldToHeadingEulerZyx.setRoll(0.0);
//  RotationQuaternion orientationWorldToHeading = RotationQuaternion(orientationWorldToHeadingEulerZyx.getUnique());
//  this->getMeasuredState().setWorldToControlOrientation(orientationWorldToHeading);
//  const RotationQuaternion orientationHeadingToBase = orientationWorldToBase*orientationWorldToHeading.inverted();
//  this->getMeasuredState().setControlToBaseOrientation(orientationHeadingToBase);

  // todo fix this in torso control
//  const Twist desiredBaseTwistInBaseFrame = Twist(orientationHeadingToBase.rotate(desiredBaseTwistInHeadingFrame_.getTranslationalVelocity()),orientationHeadingToBase.rotate(desiredBaseTwistInHeadingFrame_.getRotationalVelocity()));
//  this->getDesiredState().setBaseTwistInBaseFrame(desiredBaseTwistInBaseFrame);
  return true;
}


std::ostream& operator << (std::ostream& out, const TorsoStarlETH& torso) {
//  out << "Orientation base to world (z-y-x): " << EulerAnglesZyx(torso.getDesiredState().getWorldToBaseOrientationInWorldFrame()).getUnique() << std::endl;
//  out << "Desired speed: " << torso.getDesiredState().getBaseTwistInBaseFrame();
  return out;
}




} /* namespace loco */
