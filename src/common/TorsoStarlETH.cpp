/*
 * StateDynamicGait.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#include "loco/common/TorsoStarlETH.hpp"

namespace loco {

TorsoStarlETH::TorsoStarlETH() :
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


double TorsoStarlETH::getHeadingSpeedInBaseFrame() {
  return measuredTwistInBaseFrame_.getTranslationalVelocity().x();
}

double TorsoStarlETH::getTurningSpeedInBaseFrame() {
  return measuredTwistInBaseFrame_.getRotationalVelocity().z();
}

double TorsoStarlETH::getLateralSpeedInBaseFrame() {
  return measuredTwistInBaseFrame_.getTranslationalVelocity().y();
}

double TorsoStarlETH::getDesiredHeadingSpeedInBaseFrame() {
  return desiredTwistInBaseFrame_.getTranslationalVelocity().x();
}

double TorsoStarlETH::getDesiredTurningSpeedInBaseFrame() {
  return desiredTwistInBaseFrame_.getRotationalVelocity().z();
}

double TorsoStarlETH::getDesiredLateralSpeedInBaseFrame() {
  return desiredTwistInBaseFrame_.getTranslationalVelocity().y();
}


void TorsoStarlETH::setMeasuredTwistInBaseFrame(const Twist& twist) {
  measuredTwistInBaseFrame_ = twist;
}

void TorsoStarlETH::setDesiredTwistInBaseFrame(const Twist& twist) {
  desiredTwistInBaseFrame_ = twist;
}

const TorsoBase::Twist& TorsoStarlETH::getMeasuredTwistInBaseFrame() {
  return measuredTwistInBaseFrame_;
}
const TorsoBase::Twist& TorsoStarlETH::getDesiredTwistInBaseFrame() {
  return desiredTwistInBaseFrame_;
}

const TorsoBase::Pose& TorsoStarlETH::getMeasuredPoseInWorldFrame() {
  return measuredPoseInWorldFrame_;
}
const TorsoBase::Pose& TorsoStarlETH::getDesiredPoseInWorldFrame() {
  return desiredPoseInWorldFrame_;
}

void TorsoStarlETH::setMeasuredPoseInWorldFrame(const Pose& pose) {
  measuredPoseInWorldFrame_ = pose;
}
void TorsoStarlETH::setDesiredPoseInWorldFrame(const Pose& pose) {
  desiredPoseInWorldFrame_ = pose;
}

} /* namespace loco */
