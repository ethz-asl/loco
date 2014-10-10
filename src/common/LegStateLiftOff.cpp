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
  positionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame_()
{

}

LegStateLiftOff::~LegStateLiftOff() {

}

const Position& LegStateLiftOff::getPositionWorldToHipInWorldFrame() const {
  return hipPositionInWorldFrame_;
}

const Position& LegStateLiftOff::getPositionWorldToFootInWorldFrame() const {
  return footPositionInWorldFrame_;
}

const Position& LegStateLiftOff::getPositionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame() const {
  return positionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame_;
}

void LegStateLiftOff::setPositionWorldToHipInWorldFrame(const Position& hipPositionInWorldFrame)  {
  hipPositionInWorldFrame_ = hipPositionInWorldFrame;
}

void LegStateLiftOff::setPositionWorldToFootInWorldFrame(const Position& footPositionInWorldFrame)
{
  footPositionInWorldFrame_ = footPositionInWorldFrame;
}

void LegStateLiftOff::setPositionWorldToHipOnTerrainAlongWorldZToSurfaceAtLiftOffInWorldFrame(const Position& positionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame) {
  positionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame_ = positionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame;
}


} /* namespace loco */
