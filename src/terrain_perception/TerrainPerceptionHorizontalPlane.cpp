/*
 * TerrainPerceptionHorizontalPlane.cpp
 *
 *  Created on: Apr 15, 2014
 *      Author: gech
 */

#include "loco/terrain_perception/TerrainPerceptionHorizontalPlane.hpp"

namespace loco {

TerrainPerceptionHorizontalPlane::TerrainPerceptionHorizontalPlane(TerrainModelHorizontalPlane* terrainModel, LegGroup* legs) :
TerrainPerceptionBase(),
terrainModel_(terrainModel),
legs_(legs)
{


}

TerrainPerceptionHorizontalPlane::~TerrainPerceptionHorizontalPlane() {

}

bool TerrainPerceptionHorizontalPlane::initialize(double dt) {
  if(!advance(dt)) {
    return false;
  }
  return true;
}

bool TerrainPerceptionHorizontalPlane::advance(double dt) {
  int groundedLimbCount = 0;
  double gHeight = 0.0;


  for (auto leg : *legs_) {
    if (leg->isAndShouldBeGrounded()){
      groundedLimbCount++;
      gHeight += leg->getWorldToFootPositionInWorldFrame().z();
    }
  }

  if (groundedLimbCount > 0) {
    terrainModel_->setHeight(gHeight / groundedLimbCount);
  }
  return true;
}


} /* namespace loco */

