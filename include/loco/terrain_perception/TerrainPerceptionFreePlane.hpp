/*
 * TerrainPerceptionFreePlane.hpp
 *
 *  Created on: Aug 28, 2014
 *      Author: C. Dario Bellicoso
 */

#ifndef TERRAINPERCEPTIONFREEPLANE_HPP_
#define TERRAINPERCEPTIONFREEPLANE_HPP_

#include "loco/terrain_perception/TerrainPerceptionBase.hpp"
#include "loco/common/TerrainModelFreePlane.hpp"
#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoStarlETH.hpp"

namespace loco {
  class TerrainPerceptionFreePlane: public TerrainPerceptionBase {
   public:
    TerrainPerceptionFreePlane(TerrainModelFreePlane* terrainModel, LegGroup* legs, TorsoStarlETH* torsoStarleth);
    virtual ~TerrainPerceptionFreePlane();

    virtual bool initialize(double dt);
    virtual bool advance(double dt);

   protected:
     TerrainModelFreePlane* terrainModel_;
     LegGroup* legs_;
     TorsoStarlETH* torsoStarleth_;
     loco::Position lastFootPositionsInBaseFrame[4];

     void estimatePlane();


  }; // class
} /* namespace loco */


#endif /* TERRAINPERCEPTIONFREEPLANE_HPP_ */
