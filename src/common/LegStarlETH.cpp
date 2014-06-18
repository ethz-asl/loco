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
}

LegStarlETH::~LegStarlETH()
{

  for (auto link : *links_) {
    delete link;
  }
  delete links_;
}

const Position& LegStarlETH::getWorldToFootPositionInWorldFrame() const
{
  return positionWorldToFootInWorldFrame_;
}

const Position& LegStarlETH::getWorldToHipPositionInWorldFrame() const
{
  return positionWorldToHipInWorldFrame_;
}

const Position& LegStarlETH::getWorldToFootPositionInBaseFrame() const
{
  return positionWorldToFootInBaseFrame_;
}

const Position& LegStarlETH::getWorldToHipPositionInBaseFrame() const
{
  return positionWorldToHipInBaseFrame_;
}

const LinearVelocity& LegStarlETH::getHipLinearVelocityInWorldFrame() const
{
  return linearVelocityHipInWorldFrame_;
}

const LinearVelocity& LegStarlETH::getFootLinearVelocityInWorldFrame() const
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
  stateLiftOff_.setFootPositionInWorldFrame(positionWorldToFootInWorldFrame_);
  stateLiftOff_.setHipPositionInWorldFrame(positionWorldToHipInWorldFrame_);

  return true;
}

bool LegStarlETH::advance(double dt)
{
  properties_.advance(dt);
  
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

LegStarlETH::JointPositions LegStarlETH::getJointPositionsFromBaseToFootPositionInBaseFrame(
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


const Position& LegStarlETH::getBaseToFootPositionInBaseFrame() const
{
  return positionBaseToFootInBaseFrame_;
}

const Position& LegStarlETH::getBaseToHipPositionInBaseFrame() const {
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
