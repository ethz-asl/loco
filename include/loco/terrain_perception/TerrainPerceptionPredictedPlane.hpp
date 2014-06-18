/*
 * TerrainPerceptionPerceptedPlane.hpp
 *
 *  Created on: 31.03.2014
 *      Author: acfloria
 */

#ifndef TERRAINPERCEPTIONPREDICTEDPLANE_HPP_
#define TERRAINPERCEPTIONPREDICTEDPLANE_HPP_

#include "loco/terrain_perception/TerrainPerceptionBase.hpp"
#include "loco/common/TerrainModelPerceptedPlane.hpp"
#include "loco/common/LegGroup.hpp"
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/TorsoBase.hpp"


namespace loco {

class TerrainPerceptionPredictedPlane: public TerrainPerceptionBase  {
 public:
  TerrainPerceptionPredictedPlane(LegGroup* legs, TerrainModelPerceptedPlane* terrain, TorsoBase* torso);
  virtual ~TerrainPerceptionPredictedPlane();
  virtual bool initialize(double dt);
  //TODO create an initializer, which initializes the omega matrix of terrainBlindWalking with 4 feet

  virtual bool advance(double dt);

  /*!
   * Acessor for the terrain
   */
  virtual const TerrainModelBase* getTerrain();

 private:
  TerrainModelPerceptedPlane* terrain_;
  LegGroup* legs_;
  TorsoBase* torso_;

};

} /* namespace loco */

#endif /* TERRAINPERCEPTION_HPP_ */
