/*
 * TorsoStateDesired.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: gech
 */

#include "loco/common/TorsoStateDesired.hpp"

namespace loco {

TorsoStateDesired::TorsoStateDesired() : TorsoStateBase(),
linearVelocityBaseInControlFrame_(),
angularVelocityBaseInControlFrame_()
{

}

TorsoStateDesired::~TorsoStateDesired() {

}

void TorsoStateDesired::setLinearVelocityBaseInControlFrame(const LinearVelocity& linearVelocity) {
  linearVelocityBaseInControlFrame_ = linearVelocity;
}

const LinearVelocity& TorsoStateDesired::getLinearVelocityBaseInControlFrame() const {
  return linearVelocityBaseInControlFrame_;
}

void TorsoStateDesired::setAngularVelocityBaseInControlFrame(const LocalAngularVelocity& angularVelocity) {
  angularVelocityBaseInControlFrame_ = angularVelocity;
}

const LocalAngularVelocity& TorsoStateDesired::getAngularVelocityBaseInControlFrame() const {
  return angularVelocityBaseInControlFrame_;
}



} /* namespace loco */
