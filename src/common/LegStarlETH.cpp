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
  positionWorldToFootInWorldFrame_(Position::Zero()),
  positionWorldToHipInWorldFrame_(Position::Zero()),
  positionWorldToFootInBaseFrame_(Position::Zero()),
  positionWorldToHipInBaseFrame_(Position::Zero()),
  linearVelocityHipInWorldFrame_(Velocity::Zero())
{

}

LegStarlETH::~LegStarlETH() {

}

const LegStarlETH::Position& LegStarlETH::getWorldToFootPositionInWorldFrame() const  {
  return positionWorldToFootInWorldFrame_;
}

const LegStarlETH::Position& LegStarlETH::getWorldToHipPositionInWorldFrame() const  {
  return positionWorldToHipInWorldFrame_;
}

const LegStarlETH::Position& LegStarlETH::getWorldToFootPositionInBaseFrame() const  {
  return positionWorldToFootInBaseFrame_;
}

const LegStarlETH::Position& LegStarlETH::getWorldToHipPositionInBaseFrame() const  {
  return positionWorldToHipInBaseFrame_;
}

const LegStarlETH::Velocity& LegStarlETH::getHipLinearVelocityInWorldFrame() const {
  return linearVelocityHipInWorldFrame_;
}

void LegStarlETH::advance(double dt) {
  if (robotModel_->contacts().getCA()(iLeg_) == 1) {
    this->setIsGrounded(true);
  } else {
    this->setIsGrounded(false);
  }
  positionWorldToFootInWorldFrame_ = robotModel_->kin().getJacobianTByLeg_World2Foot_CSw(iLeg_)->getPos();
  positionWorldToHipInWorldFrame_ = robotModel_->kin().getJacobianTByLeg_World2Hip_CSw(iLeg_)->getPos();
  linearVelocityHipInWorldFrame_ = robotModel_->kin().getJacobianTByLeg_World2Hip_CSw(iLeg_)->getVel();

  const Eigen::Vector4d vQuat = robotModel_->est().getActualEstimator()->getQuat();
  RotationQuaternion rquatWorldToBase(vQuat(0), vQuat(1), vQuat(2), vQuat(3));
  rquatWorldToBase.invert();
  positionWorldToFootInBaseFrame_ = rquatWorldToBase.rotate(positionWorldToFootInWorldFrame_);
  positionWorldToHipInBaseFrame_ = rquatWorldToBase.rotate(positionWorldToHipInWorldFrame_);

}


LegStarlETH::JointPositions LegStarlETH::getJointPositionsFromBaseToFootPositionInBaseFrame(const Position& positionBaseToFootInBaseFrame) {
 return JointPositions(robotModel_->kin().getJointPosFromFootPosCSmb(positionBaseToFootInBaseFrame, iLeg_));
}

LegPropertiesBase& LegStarlETH::getProperties() {
 return static_cast<LegPropertiesBase&>(properties_);
}

} /* namespace loco */
