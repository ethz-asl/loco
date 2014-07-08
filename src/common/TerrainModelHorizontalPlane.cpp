/*
 * TerrainModelHorizontalPlane.cpp
 *
 *  Created on: Apr 4, 2014
 *      Author: gech
 */

#include "loco/common/TerrainModelHorizontalPlane.hpp"

namespace loco {

TerrainModelHorizontalPlane::TerrainModelHorizontalPlane() :
    TerrainModelBase(),
    heightInWorldFrame_(0.0),
    normalInWorldFrame_(loco::Vector::UnitZ())
{


}

TerrainModelHorizontalPlane::~TerrainModelHorizontalPlane() {

}

bool TerrainModelHorizontalPlane::initialize(double dt) {
  heightInWorldFrame_ = 0.0;
  return true;
}


bool TerrainModelHorizontalPlane::getNormal(const loco::Position& positionWorldToLocationInWorldFrame, loco::Vector& normalInWorldFrame) const
{
  normalInWorldFrame = normalInWorldFrame_;
  return true;
}

bool TerrainModelHorizontalPlane::getHeight(loco::Position& positionWorldToLocationInWorldFrame) const
{
  positionWorldToLocationInWorldFrame.z() = heightInWorldFrame_;
  return false;
}


bool TerrainModelHorizontalPlane::getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const {
  heightInWorldFrame = heightInWorldFrame_;
  return true;
}

void TerrainModelHorizontalPlane::setHeight(double heightInWorldFrame) {
  heightInWorldFrame_ = heightInWorldFrame;
}


} /* namespace loco */
