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

const LegStarlETH::Velocity& LegStarlETH::getHipLinearVelocityInWorldFrame() const {
  return linearVelocityHipInWorldFrame_;
}

void LegStarlETH::advance(double dt) {
  positionWorldToFootInWorldFrame_ = robotModel_->kin().getJacobianTByLeg_World2Foot_CSw(iLeg_)->getPos();
  positionWorldToHipInWorldFrame_ = robotModel_->kin().getJacobianTByLeg_World2Hip_CSw(iLeg_)->getPos();
  linearVelocityHipInWorldFrame_ = robotModel_->kin().getJacobianTByLeg_World2Hip_CSw(iLeg_)->getVel();
}


const LegStarlETH::JointPositions& LegStarlETH::getJointPositionsFromBaseToFootPositionInBaseFrame(const Position& positionBaseToFootInBaseFrame) {
 return JointPositions(robotModel_->kin().getJointPosFromFootPosCSmb(positionBaseToFootInBaseFrame, iLeg_));
}

} /* namespace loco */
