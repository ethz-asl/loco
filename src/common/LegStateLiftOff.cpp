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

const Position& LegStateLiftOff::getHipPositionInWorldFrame() const {
  return hipPositionInWorldFrame_;
}

const Position& LegStateLiftOff::getFootPositionInWorldFrame() const {
  return footPositionInWorldFrame_;
}

const Position& LegStateLiftOff::getPositionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame() const {
  return positionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame_;
}

void LegStateLiftOff::setWorldToHipPositionInWorldFrame(const Position& hipPositionInWorldFrame)  {
  hipPositionInWorldFrame_ = hipPositionInWorldFrame;
}

void LegStateLiftOff::setWorldToFootPositionInWorldFrame(const Position& footPositionInWorldFrame)
{
  footPositionInWorldFrame_ = footPositionInWorldFrame;
}

void LegStateLiftOff::setPositionWorldToHipOnTerrainAlongWorldZToSurfaceAtLiftOffInWorldFrame(const Position& positionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame) {
  positionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame_ = positionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame;
}


} /* namespace loco */
