/*
 * TerrainPerceptionPredictedPlane.cpp
 *
 *  Created on: 31.03.2014
 *      Author: acfloria
 */

#include "loco/terrain_perception/TerrainPerceptionPredictedPlane.hpp"

namespace loco {

TerrainPerceptionPredictedPlane::TerrainPerceptionPredictedPlane(LegGroup* legs, TerrainModelPerceptedPlane* terrain, TorsoBase* torso) :
TerrainPerceptionBase(),
legs_(legs),
terrain_(terrain),
torso_(torso)
{


}

TerrainPerceptionPredictedPlane::~TerrainPerceptionPredictedPlane() {
  // TODO Auto-generated destructor stub
}



bool TerrainPerceptionPredictedPlane::initialize(double dt) {
  if (!terrain_->initialize(dt)) {
    return false;
  }
  if(!advance(dt)) {
    return false;
  }
  return true;
}


bool TerrainPerceptionPredictedPlane::advance(double dt)
{
  //update Plane
  for (auto leg : *legs_) {
      if(leg->getStateTouchDown()->isNow())
      {
//        std::cout << leg->getName() << std::endl;
//        std::cout << "leg position: " << leg->getWorldToFootPositionInWorldFrame() << std::endl;
        terrain_->updatePlane(leg->getWorldToFootPositionInWorldFrame());
      }
    }



  //update angles
  terrain_->updateAngles(torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame());
  return true;
}

const TerrainModelBase* TerrainPerceptionPredictedPlane::getTerrain() {
  return terrain_;
}

} /* namespace loco */
