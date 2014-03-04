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

TorsoPropertiesBase& TorsoStarlETH::getProperties() {
  return static_cast<TorsoPropertiesBase&>(properties_);
}

void TorsoStarlETH::advance(double dt) {
  const Eigen::Vector4d vQuat = robotModel_->est().getActualEstimator()->getQuat();
  RotationQuaternion rquatWorldToBase(vQuat(0), vQuat(1), vQuat(2), vQuat(3));
  rquatWorldToBase.invert();
  const Position position(rquatWorldToBase.rotate(robotModel_->kin()[robotModel::JT_World2Base_CSw]->getPos()));
//  std::cout << "world2base position: " << robotModel_->kin()[robotModel::JT_World2Base_CSw]->getPos() << std::endl;
//  std::cout << "position: " << rquatWorldToBase.rotate(robotModel_->kin()[robotModel::JT_World2Base_CSw]->getPos()) << std::endl;

  this->getMeasuredState().setWorldToBasePoseInWorldFrame(Pose(position, rquatWorldToBase));
  const LinearVelocity linearVelocity(rquatWorldToBase.rotate(robotModel_->kin()[robotModel::JT_World2Base_CSw]->getVel()));
  const LocalAngularVelocity localAngularVelocity(rquatWorldToBase.rotate(robotModel_->kin()(robotModel::JR_World2Base_CSw)->getOmega()));
  this->getMeasuredState().setBaseTwistInBaseFrame(Twist(linearVelocity, localAngularVelocity));
}




} /* namespace loco */
