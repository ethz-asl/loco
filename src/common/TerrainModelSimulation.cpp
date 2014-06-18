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
    : terrain_(terrain)
{

}

TerrainModelSimulation::~TerrainModelSimulation()
{

}

bool TerrainModelSimulation::getNormal(const loco::Position& position, loco::Vector& normal) const
{
  robotModel::VectorP tempPosition = position.vector();
  terrain_->getNormal(tempPosition, normal.vector());
  return true;
}

bool TerrainModelSimulation::getHeight(loco::Position& position) const
{
  robotModel::VectorP tempPosition = position.vector();
  if(!terrain_->getHeight(tempPosition)) return false;
  position.z() = tempPosition.z();
  return true;
}

bool TerrainModelSimulation::initialize(double dt) {

  return true;
}

} /* namespace loco */
