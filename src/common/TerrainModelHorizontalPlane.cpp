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
    normalInWorldFrame_(loco::Vector::UnitZ()),
    frictionCoefficientBetweenTerrainAndFoot_(0.8)
{


}

TerrainModelHorizontalPlane::~TerrainModelHorizontalPlane() {

}

bool TerrainModelHorizontalPlane::initialize(double dt) {
  heightInWorldFrame_ = 0.0;
  normalInWorldFrame_ = loco::Vector::UnitZ();
  frictionCoefficientBetweenTerrainAndFoot_ = 0.6;
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
  return true;
}


bool TerrainModelHorizontalPlane::getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const {
  heightInWorldFrame = heightInWorldFrame_;
  return true;
}

void TerrainModelHorizontalPlane::setHeight(double heightInWorldFrame) {
  heightInWorldFrame_ = heightInWorldFrame;
}

bool TerrainModelHorizontalPlane::getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const {
  frictionCoefficient = frictionCoefficientBetweenTerrainAndFoot_;
  return true;
}

Position TerrainModelHorizontalPlane::getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(const Position& positionInWorldFrame)  const {
  Position position = positionInWorldFrame;
  getHeight(position);
  return position;
}


double TerrainModelHorizontalPlane::getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(const Position& positionInWorldFrame) const {
  double distance;
  getHeight(positionInWorldFrame, distance);
  return distance;
}


} /* namespace loco */
