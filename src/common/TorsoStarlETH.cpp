/*
 * StateDynamicGait.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#include "loco/common/TorsoStarlETH.hpp"

namespace loco {

TorsoStarlETH::TorsoStarlETH(robotModel::RobotModel* robotModel) :
    robotModel_(robotModel),
    stridePhase_(0.0)
{


}

TorsoStarlETH::~TorsoStarlETH() {

}

double TorsoStarlETH::getStridePhase() {
  return stridePhase_;
}

void TorsoStarlETH::setStridePhase(double stridePhase) {
  stridePhase_ = stridePhase;
}

TorsoStateMeasured& TorsoStarlETH::getMeasuredState() {
  return stateMeasured_;
}

TorsoStateDesired& TorsoStarlETH::getDesiredState() {
  return stateDesired_;
}

void TorsoStarlETH::advance(double dt) {
  const Eigen::Vector4d vQuat = robotModel_->est().getActualEstimator()->getQuat();
  TorsoBase::Pose::Rotation rquatWorldToBase(vQuat(0), vQuat(1), vQuat(2), vQuat(3));
  rquatWorldToBase.invert();
  const TorsoBase::Pose::Position position(rquatWorldToBase.rotate(robotModel_->kin()[robotModel::JT_World2Base_CSw]->getPos()));
//  std::cout << "world2base position: " << robotModel_->kin()[robotModel::JT_World2Base_CSw]->getPos() << std::endl;
//  std::cout << "position: " << rquatWorldToBase.rotate(robotModel_->kin()[robotModel::JT_World2Base_CSw]->getPos()) << std::endl;

  this->getMeasuredState().setWorldToBasePoseInWorldFrame(TorsoBase::Pose(position, rquatWorldToBase));
  const TorsoBase::Twist::PositionDiff linearVelocity(rquatWorldToBase.rotate(robotModel_->kin()[robotModel::JT_World2Base_CSw]->getVel()));
  const TorsoBase::Twist::RotationDiff localAngularVelocity(rquatWorldToBase.rotate(robotModel_->kin()(robotModel::JR_World2Base_CSw)->getOmega()));
  this->getMeasuredState().setBaseTwistInBaseFrame(TorsoBase::Twist(linearVelocity, localAngularVelocity));
}




} /* namespace loco */
