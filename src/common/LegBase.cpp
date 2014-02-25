/*
 * LegBase.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#include "loco/common/LegBase.hpp"

namespace loco {

LegBase::LegBase() :
    stancePhase_(0.0),
    swingPhase_(0.0),
    stanceDuration_(0.0),
    swingDuration_(0.0),
    isInStanceMode_(false),
    isInSwingMode_(true),
    isGrounded_(false),
    shouldBeGrounded_(false)
{

}

LegBase::~LegBase() {

}


double LegBase::getStancePhase() {
  return stancePhase_;
}
double LegBase::getSwingPhase() {
  return swingPhase_;
}

double LegBase::getStanceDuration() {
  return stanceDuration_;
}
double LegBase::getSwingDuration() {
  return swingDuration_;
}

bool LegBase::isInStanceMode() {
  return isInStanceMode_;
}
bool LegBase::isInSwingMode() {
  return isInSwingMode_;
}

bool LegBase::isGrounded()  {
  return isGrounded_;
}
bool LegBase::shouldBeGrounded() {
  return shouldBeGrounded_;
}
bool LegBase::isAndShoulBeGrounded() {
  return (isGrounded_ && shouldBeGrounded_);
}

void LegBase::setStancePhase(double phase) {
  stancePhase_ = phase;
}

void LegBase::setSwingPhase(double phase) {
  swingPhase_ = phase;
}

void LegBase::setStanceDuration(double duration) {
  stanceDuration_ = duration;
}

void LegBase::setSwingDuration(double duration) {
  swingDuration_ = duration;
}

void LegBase::setIsInStanceMode(bool isInStanceMode) {
  isInStanceMode_ = isInStanceMode;
}

void LegBase::setIsInSwingMode(bool isInSwingMode) {
  isInSwingMode_ = isInSwingMode;
}

void LegBase::setIsGrounded(bool isGrounded) {
  isGrounded_ = isGrounded;
}

void LegBase::setShouldBeGrounded(bool shouldBeGrounded) {
  shouldBeGrounded_ = shouldBeGrounded;
}



} /* namespace loco */
