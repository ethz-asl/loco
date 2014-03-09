/*
 * LegStarlETH.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#include "loco/common/LegStarlETH.hpp"

namespace loco {

LegStarlETH::LegStarlETH(const std::string& name, int iLeg,  robotModel::RobotModel* robotModel) :
  LegBase(name),
  iLeg_(iLeg),
  robotModel_(robotModel),
  positionWorldToFootInWorldFrame_(),
  positionWorldToHipInWorldFrame_(),
  positionWorldToFootInBaseFrame_(),
  positionWorldToHipInBaseFrame_(),
  positionBaseToFootInBaseFrame_(),
  positionBaseToHipInBaseFrame_(),
  linearVelocityHipInWorldFrame_()
{
  desiredJointControlModes_.setConstant(robotModel::AM_Velocity);
  translationJacobianBaseToFootInBaseFrame_.setZero();
}

LegStarlETH::~LegStarlETH() {

}

const Position& LegStarlETH::getWorldToFootPositionInWorldFrame() const  {
  return positionWorldToFootInWorldFrame_;
}

const Position& LegStarlETH::getWorldToHipPositionInWorldFrame() const  {
  return positionWorldToHipInWorldFrame_;
}

const Position& LegStarlETH::getWorldToFootPositionInBaseFrame() const  {
  return positionWorldToFootInBaseFrame_;
}

const Position& LegStarlETH::getWorldToHipPositionInBaseFrame() const  {
  return positionWorldToHipInBaseFrame_;
}

const LinearVelocity& LegStarlETH::getHipLinearVelocityInWorldFrame() const {
  return linearVelocityHipInWorldFrame_;
}

const LegBase::TranslationJacobian& LegStarlETH::getTranslationJacobianFromBaseToFootInBaseFrame() const
{
  return translationJacobianBaseToFootInBaseFrame_;
}


void LegStarlETH::advance(double dt) {
  if (robotModel_->contacts().getCA()(iLeg_) == 1) {
    this->setIsGrounded(true);
  } else {
    this->setIsGrounded(false);
  }
  positionWorldToFootInWorldFrame_ = Position(robotModel_->kin().getJacobianTByLeg_World2Foot_CSw(iLeg_)->getPos());
  positionWorldToHipInWorldFrame_ = Position(robotModel_->kin().getJacobianTByLeg_World2Hip_CSw(iLeg_)->getPos());
  linearVelocityHipInWorldFrame_ = LinearVelocity(robotModel_->kin().getJacobianTByLeg_World2Hip_CSw(iLeg_)->getVel());

  const Eigen::Vector4d vQuat = robotModel_->est().getActualEstimator()->getQuat();
  RotationQuaternion rquatWorldToBase(vQuat(0), vQuat(1), vQuat(2), vQuat(3));
  rquatWorldToBase.invert();
  positionWorldToFootInBaseFrame_ = rquatWorldToBase.rotate(positionWorldToFootInWorldFrame_);
  positionWorldToHipInBaseFrame_ = rquatWorldToBase.rotate(positionWorldToHipInWorldFrame_);

  this->setMeasuredJointPositions(robotModel_->q().getQj().block<3,1>(iLeg_*nJoints_,0));

  positionBaseToFootInBaseFrame_ = Position(robotModel_->kin().getJacobianTByLeg_Base2Foot_CSmb(iLeg_)->getPos());
  positionBaseToHipInBaseFrame_ = Position(robotModel_->kin().getJacobianTByLeg_Base2Hip_CSmb(iLeg_)->getPos());

  translationJacobianBaseToFootInBaseFrame_ = robotModel_->kin().getJacobianTByLeg_Base2Foot_CSmb(iLeg_)
      ->getJ().block<nDofContactPoint_, nJoints_>(0, RM_NQB + iLeg_*nJoints_);
}


LegStarlETH::JointPositions LegStarlETH::getJointPositionsFromBaseToFootPositionInBaseFrame(const Position& positionBaseToFootInBaseFrame) {
 return JointPositions(robotModel_->kin().getJointPosFromFootPosCSmb(positionBaseToFootInBaseFrame.toImplementation(), iLeg_));
}

LegPropertiesBase& LegStarlETH::getProperties() {
 return static_cast<LegPropertiesBase&>(properties_);
}

const Position& LegStarlETH::getBaseToFootPositionInBaseFrame() const {
  return positionBaseToFootInBaseFrame_;
}

const Position& LegStarlETH::getBaseToHipPositionInBaseFrame() const {
  return positionBaseToHipInBaseFrame_;
}

} /* namespace loco */
