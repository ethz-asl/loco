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
  footPositionInWorldFrame_(Position::Zero()),
  hipPositionInWorldFrame_(Position::Zero())
{

}

LegStateLiftOff::~LegStateLiftOff() {

}

const LegStateLiftOff::Position& LegStateLiftOff::getHipPositionInWorldFrame() const {
  return hipPositionInWorldFrame_;
}

const LegStateLiftOff::Position& LegStateLiftOff::getFootPositionInWorldFrame() const {
  return footPositionInWorldFrame_;
}

void LegStateLiftOff::setHipPositionInWorldFrame(const LegStateLiftOff::Position& hipPositionInWorldFrame)  {
  hipPositionInWorldFrame_ = hipPositionInWorldFrame;
}

void LegStateLiftOff::setFootPositionInWorldFrame(const LegStateLiftOff::Position& footPositionInWorldFrame)
{
  footPositionInWorldFrame_ = footPositionInWorldFrame;
}

} /* namespace loco */
