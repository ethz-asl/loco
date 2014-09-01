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

  enum EstimatePlaneInFrame {World, Base};

  class TerrainPerceptionFreePlane: public TerrainPerceptionBase {
   public:
    TerrainPerceptionFreePlane(TerrainModelFreePlane* terrainModel, LegGroup* legs, TorsoStarlETH* torso, loco::EstimatePlaneInFrame estimateFrame = loco::EstimatePlaneInFrame::Base);
    virtual ~TerrainPerceptionFreePlane();

    /*! Initialize foot measurements.
     * @param dt  time step [s]
     */
    virtual bool initialize(double dt);

    /*! Advance in time.
     * @param dt  time step [s]
     */
    virtual bool advance(double dt);

   protected:
     TerrainModelFreePlane* terrainModel_;
     LegGroup* legs_;
     TorsoStarlETH* torso_;
     loco::Position mostRecentPositionOfFoot_[4];
     loco::EstimatePlaneInFrame estimatePlaneInFrame_;
     int numberOfLegs_;

     /*! Estimate the free plane parameters and update the terrain model.
      */
     void updatePlaneEstimation();


  }; // class
} /* namespace loco */


#endif /* TERRAINPERCEPTIONFREEPLANE_HPP_ */
