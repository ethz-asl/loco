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
 * LegStarlETH.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#include "loco/common/LegStarlETH.hpp"

#include "loco/common/LegLinkGroup.hpp"

namespace loco {
//LegStarlETH::LegStarlETH(const std::string& name, int iLeg, LegLinkGroup* links,  robotModel::RobotModel* robotModel) :
LegStarlETH::LegStarlETH(const std::string& name, int iLeg, robotModel::RobotModel* robotModel) :
  LegBase(name, new LegLinkGroup),
  iLeg_(iLeg),
  robotModel_(robotModel),
  properties_(iLeg, robotModel),
  positionWorldToFootInWorldFrame_(),
  positionWorldToHipInWorldFrame_(),
  positionWorldToFootInBaseFrame_(),
  positionWorldToHipInBaseFrame_(),
  positionBaseToFootInBaseFrame_(),
  positionBaseToHipInBaseFrame_(),
  linearVelocityHipInWorldFrame_(),
  forceFootContactInWorldFrame_(),
  normalFootContactInWorldFrame_()
{
  links_->addLegLink(new LegLink);
  links_->addLegLink(new LegLink);
  links_->addLegLink(new LegLink);
  desiredJointControlModes_.setConstant(robotModel::AM_Velocity);
  translationJacobianBaseToFootInBaseFrame_.setZero();

  links_->getLegLink(0)->setMass(robotModel_->params().hip_.m);
  links_->getLegLink(1)->setMass(robotModel_->params().thigh_.m);
  links_->getLegLink(2)->setMass(robotModel_->params().shank_.m);


  stateSwitcher_ = new StateSwitcher();

}

LegStarlETH::~LegStarlETH()
{

  for (auto link : *links_) {
    delete link;
  }
  delete links_;

  delete stateSwitcher_;

}

const Position& LegStarlETH::getPositionWorldToFootInWorldFrame() const
{
  return positionWorldToFootInWorldFrame_;
}

const Position& LegStarlETH::getPositionWorldToHipInWorldFrame() const
{
  return positionWorldToHipInWorldFrame_;
}

const Position& LegStarlETH::getPositionWorldToFootInBaseFrame() const
{
  return positionWorldToFootInBaseFrame_;
}

const Position& LegStarlETH::getPositionWorldToHipInBaseFrame() const
{
  return positionWorldToHipInBaseFrame_;
}

const LinearVelocity& LegStarlETH::getLinearVelocityHipInWorldFrame() const
{
  return linearVelocityHipInWorldFrame_;
}

const LinearVelocity& LegStarlETH::getLinearVelocityFootInWorldFrame() const
{
  return linearVelocityFootInWorldFrame_;
}

const LegBase::TranslationJacobian& LegStarlETH::getTranslationJacobianFromBaseToFootInBaseFrame() const
{
  return translationJacobianBaseToFootInBaseFrame_;
}




bool LegStarlETH::initialize(double dt) {



  if(!this->advance(dt)) {
    return false;
  }
  stateLiftOff_.setPositionWorldToFootInWorldFrame(positionWorldToFootInWorldFrame_);
  stateLiftOff_.setPositionWorldToHipInWorldFrame(positionWorldToHipInWorldFrame_);

  stateTouchDown_.setTouchdownFootPositionInWorldFrame(positionWorldToFootInWorldFrame_);

  stateSwitcher_->initialize(0);

  return true;
}

bool LegStarlETH::advance(double dt)
{
  properties_.advance(dt);
  
  this->setWasGrounded(this->isGrounded());

  if (robotModel_->contacts().getCA()(iLeg_) == 1) {
    this->setIsGrounded(true);
  } else {
    this->setIsGrounded(false);
  }
  positionWorldToFootInWorldFrame_ = Position(robotModel_->kin().getJacobianTByLeg_World2Foot_CSw(iLeg_)->getPos());
  positionWorldToHipInWorldFrame_ = Position(robotModel_->kin().getJacobianTByLeg_World2Hip_CSw(iLeg_)->getPos());
  linearVelocityHipInWorldFrame_ = LinearVelocity(robotModel_->kin().getJacobianTByLeg_World2Hip_CSw(iLeg_)->getVel());
  linearVelocityFootInWorldFrame_ = LinearVelocity(robotModel_->kin().getJacobianTByLeg_World2Foot_CSw(iLeg_)->getVel());

  const Eigen::Vector4d vQuat = robotModel_->est().getActualEstimator()->getQuat();
  RotationQuaternion rquatWorldToBase(vQuat(0), vQuat(1), vQuat(2), vQuat(3));
  rquatWorldToBase.invert();
  positionWorldToFootInBaseFrame_ = rquatWorldToBase.rotate(positionWorldToFootInWorldFrame_);
  positionWorldToHipInBaseFrame_ = rquatWorldToBase.rotate(positionWorldToHipInWorldFrame_);

  this->setMeasuredJointPositions(robotModel_->q().getQj().block<3, 1>(iLeg_ * nJoints_, 0));
  this->setMeasuredJointVelocities(robotModel_->q().getdQj().block<3, 1>(iLeg_ * nJoints_, 0));
  this->setMeasuredJointTorques(robotModel_->sensors().getJointTorques().block<3, 1>(iLeg_ * nJoints_, 0));

  positionBaseToFootInBaseFrame_ = Position(robotModel_->kin().getJacobianTByLeg_Base2Foot_CSmb(iLeg_)->getPos());
  positionBaseToHipInBaseFrame_ = Position(robotModel_->kin().getJacobianTByLeg_Base2Hip_CSmb(iLeg_)->getPos());

  translationJacobianBaseToFootInBaseFrame_ = robotModel_->kin().getJacobianTByLeg_Base2Foot_CSmb(iLeg_)
      ->getJ().block<nDofContactPoint_, nJoints_>(0, RM_NQB + iLeg_*nJoints_);

  links_->getLegLink(0)->setTranslationJacobianBaseToCoMInBaseFrame(robotModel_->kin().getJacobianTByLeg_Base2HipCoG_CSmb(iLeg_)
          ->getJ().block<nDofContactPoint_, nJoints_>(0, RM_NQB + iLeg_*nJoints_));
  links_->getLegLink(0)->setBaseToCoMPositionInBaseFrame(Position(robotModel_->kin().getJacobianTByLeg_Base2HipCoG_CSmb(iLeg_)->getPos()));

  links_->getLegLink(1)->setTranslationJacobianBaseToCoMInBaseFrame(robotModel_->kin().getJacobianTByLeg_Base2ThighCoG_CSmb(iLeg_)
          ->getJ().block<nDofContactPoint_, nJoints_>(0, RM_NQB + iLeg_*nJoints_));
  links_->getLegLink(1)->setBaseToCoMPositionInBaseFrame(Position(robotModel_->kin().getJacobianTByLeg_Base2ThighCoG_CSmb(iLeg_)->getPos()));

  links_->getLegLink(2)->setTranslationJacobianBaseToCoMInBaseFrame(robotModel_->kin().getJacobianTByLeg_Base2ShankCoG_CSmb(iLeg_)
          ->getJ().block<nDofContactPoint_, nJoints_>(0, RM_NQB + iLeg_*nJoints_));
  links_->getLegLink(2)->setBaseToCoMPositionInBaseFrame(Position(robotModel_->kin().getJacobianTByLeg_Base2ShankCoG_CSmb(iLeg_)->getPos()));

  forceFootContactInWorldFrame_ = Force(robotModel_->sensors().getContactForceCSw(iLeg_));
  normalFootContactInWorldFrame_ = Vector(robotModel_->sensors().getContactNormalCSw(iLeg_));
  return true;
}

LegStarlETH::JointPositions LegStarlETH::getJointPositionsFromPositionBaseToFootInBaseFrame(
    const Position& positionBaseToFootInBaseFrame)
{
  return JointPositions(
      robotModel_->kin().getJointPosFromFootPosCSmb(positionBaseToFootInBaseFrame.toImplementation(), iLeg_));
}

LegPropertiesBase& LegStarlETH::getProperties()
{
//  return static_cast<LegPropertiesBase&>(properties_);
  return properties_;
}

const LegPropertiesBase& LegStarlETH::getProperties() const
{
  return properties_;
}

const Position& LegStarlETH::getPositionBaseToFootInBaseFrame() const
{
  return positionBaseToFootInBaseFrame_;
}

const Position& LegStarlETH::getPositionBaseToHipInBaseFrame() const {
  return positionBaseToHipInBaseFrame_;
}


const Force& LegStarlETH::getFootContactForceInWorldFrame() const {
  return forceFootContactInWorldFrame_;
}

const Vector& LegStarlETH::getFootContactNormalInWorldFrame() const {
  return normalFootContactInWorldFrame_;
}


int LegStarlETH::getId() const {
  return iLeg_;
}


} /* namespace loco */
