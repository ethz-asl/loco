/*
 * TerrainModelSimulation.cpp
 *
 *  Created on: May 5, 2014
 *      Author: Peter Fankhauser
 */

#include "loco/common/TerrainModelSimulation.hpp"
#include "RobotModel.hpp"

namespace loco {

TerrainModelSimulation::TerrainModelSimulation(robotTerrain::TerrainBase* terrain)
    : TerrainModelBase(),
      terrain_(terrain)
{

}

TerrainModelSimulation::~TerrainModelSimulation()
{

}

bool TerrainModelSimulation::initialize(double dt) {
  return true;
}

bool TerrainModelSimulation::getNormal(const loco::Position& position, loco::Vector& normal) const
{
  robotModel::VectorP tempPosition = position.vector();
  terrain_->getNormal(tempPosition, normal.vector());
  return true;
}

bool TerrainModelSimulation::getHeight(loco::Position& positionWorldToLocationInWorldFrame) const
{
  robotModel::VectorP tempPosition = positionWorldToLocationInWorldFrame.vector();
  if(!terrain_->getHeight(tempPosition)) {
    positionWorldToLocationInWorldFrame.z() = 0.0;
    return false;
  }
  positionWorldToLocationInWorldFrame.z() = tempPosition.z();
  return true;
}

bool TerrainModelSimulation::getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const {
  robotModel::VectorP tempPosition = positionWorldToLocationInWorldFrame.vector();
  if(!terrain_->getHeight(tempPosition)) {
    heightInWorldFrame = 0.0;
    return false;
  }
  heightInWorldFrame = tempPosition.z();
  return true;
}

bool TerrainModelSimulation::getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const {
  assert(false); // not yet implemented!
  return false;
}




} /* namespace loco */
