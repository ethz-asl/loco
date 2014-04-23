/*
 * LegStateLiftOff.cpp
 *
 *  Created on: Feb 26, 2014
 *      Author: gech
 */

#include "loco/common/LegStateLiftOff.hpp"

namespace loco {

LegStateLiftOff::LegStateLiftOff() :
  LegStateBase(),
  footPositionInWorldFrame_(),
  hipPositionInWorldFrame_(),
  isNow_(false)
{

}

LegStateLiftOff::~LegStateLiftOff() {

}

const Position& LegStateLiftOff::getHipPositionInWorldFrame() const {
  return hipPositionInWorldFrame_;
}

const Position& LegStateLiftOff::getFootPositionInWorldFrame() const {
  return footPositionInWorldFrame_;
}

void LegStateLiftOff::setHipPositionInWorldFrame(const Position& hipPositionInWorldFrame)  {
  hipPositionInWorldFrame_ = hipPositionInWorldFrame;
}

void LegStateLiftOff::setFootPositionInWorldFrame(const Position& footPositionInWorldFrame)
{
  footPositionInWorldFrame_ = footPositionInWorldFrame;
}

void LegStateLiftOff::setIsNow(bool isNow) {
  isNow_= isNow;
}

bool LegStateLiftOff::isNow() const {
  return isNow_;
}

} /* namespace loco */
