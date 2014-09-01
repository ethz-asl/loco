/*
 * TerrainPerceptionFreePlane.hpp
 *
 *  Created on: Aug 28, 2014
 *      Author: C. Dario Bellicoso
 */

#ifndef LOCO_TERRAINPERCEPTIONFREEPLANE_HPP_
#define LOCO_TERRAINPERCEPTIONFREEPLANE_HPP_

#include "loco/terrain_perception/TerrainPerceptionBase.hpp"
#include "loco/common/TerrainModelFreePlane.hpp"
#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoStarlETH.hpp"

namespace loco {

  class TerrainPerceptionFreePlane: public TerrainPerceptionBase {

   public:
    enum EstimatePlaneInFrame {World, Base};

    TerrainPerceptionFreePlane(TerrainModelFreePlane* terrainModel,
                               LegGroup* legs,
                               TorsoStarlETH* torso,
                               TerrainPerceptionFreePlane::EstimatePlaneInFrame estimateFrame = TerrainPerceptionFreePlane::EstimatePlaneInFrame::World);
    virtual ~TerrainPerceptionFreePlane();

    /*! Initialize saved foot measurements.
     * @param dt  time step [s]
     */
    virtual bool initialize(double dt);

    /*! Advance in time. Update saved foot measurements with the latest foot positions
     * if foots are grounded.
     * @param dt  time step [s]
     */
    virtual bool advance(double dt);

   protected:
     TerrainModelFreePlane* terrainModel_;
     LegGroup* legs_;
     TorsoStarlETH* torso_;
     std::vector<loco::Position> mostRecentPositionOfFoot_;
     TerrainPerceptionFreePlane::EstimatePlaneInFrame estimatePlaneInFrame_;
     int numberOfLegs_;

     /*! Estimate the free plane parameters and update the terrain model.
      */
     void updatePlaneEstimation();

  }; // class
} /* namespace loco */


#endif /* LOCO_TERRAINPERCEPTIONFREEPLANE_HPP_ */
