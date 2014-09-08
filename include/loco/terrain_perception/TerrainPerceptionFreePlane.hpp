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
                               TerrainPerceptionFreePlane::EstimatePlaneInFrame estimateFrame = TerrainPerceptionFreePlane::EstimatePlaneInFrame::Base);
    virtual ~TerrainPerceptionFreePlane();

    /*! Initialize saved foot measurements.
     * @param dt  time step [s]
     */
    virtual bool initialize(double dt);

    /*! Advance in time. Update saved foot measurements with the latest foot positions
     * if foots are grounded. Also check if a given foot was grounded at least once.
     * @param dt  time step [s]
     */
    virtual bool advance(double dt);

    /*! Change the coordinate system of a position vector from base to world frame
     * @params[in/out] position A position vector, expressed in base frame, that has to be expressed in world frame
     */
    void rotatePassiveFromBaseToWorldFrame(loco::Position& position);

    /*! Change the coordinate system of a position vector from base to world frame taking into account
     *  the current displacement and rotation between the reference systems
     * @params[in/out] position A position vector, expressed in base frame, that has to be expressed in world frame
     */
    void homogeneousTransformFromBaseToWorldFrame(loco::Position& position);

    /*! Change the coordinate system of the foot position vector from base to world frame taking into account
     * the last saved displacement and orientation between the reference systems for the given foot
     * @params[in/out] position A position vector, expressed in base frame, that has to be expressed in world frame
     */
    void homogeneousTransformFromBaseToWorldFrame(loco::Position& position, int footID);

    /*! Get the most recent position of foot in world frame
     * @param[in/out] positionInWorldFrame Position of the foot in world frame
     * @param[in] footID Integer ID of the foot
     */
    void getMostRecentPositionOfFootInWorldFrame(loco::Position& footPositionInWorldFrame, int footID);

   protected:
     TerrainModelFreePlane* terrainModel_;
     LegGroup* legs_;
     TorsoStarlETH* torso_;
     std::vector<loco::Position> mostRecentPositionOfFoot_;
     std::vector<loco::Position> lastWorldToBasePositionInWorldFrameForFoot_;
     std::vector<RotationQuaternion> lastWorldToBaseOrientationForFoot_;
     std::vector<bool> gotFirstTouchDownOfFoot_;
     TerrainPerceptionFreePlane::EstimatePlaneInFrame estimatePlaneInFrame_;
     int numberOfLegs_;

     /*! Estimate the free plane parameters and update the terrain model.
      */
     void updatePlaneEstimation();

  }; // class
} /* namespace loco */


#endif /* LOCO_TERRAINPERCEPTIONFREEPLANE_HPP_ */
