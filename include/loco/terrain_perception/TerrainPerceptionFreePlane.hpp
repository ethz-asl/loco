/*******************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Christian Gehring, Péter Fankhauser, C. Dario Bellicoso, Stelian Coros
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
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

#include "robotUtils/filters/FirstOrderFilter.hpp"

namespace loco {

  class TerrainPerceptionFreePlane: public TerrainPerceptionBase {

   public:
    /*! Choose whether to estimate the measuring the footsteps in world or in base frame */
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


    void updateControlFrameOrigin();
    void updateControlFrameAttitude();

    std::vector<double> planeParameters_;

   protected:
     TerrainModelFreePlane* terrainModel_;
     LegGroup* legs_;
     TorsoStarlETH* torso_;
     std::vector<loco::Position> mostRecentPositionOfFoot_;
     std::vector<loco::Position> lastWorldToBasePositionInWorldFrameForFoot_;
     std::vector<RotationQuaternion> lastWorldToBaseOrientationForFoot_;
     std::vector<bool> gotFirstTouchDownOfFoot_;
     TerrainPerceptionFreePlane::EstimatePlaneInFrame estimatePlaneInFrame_;

     //--- First order filters
     robotUtils::FirstOrderFilter filterNormalX_;
     robotUtils::FirstOrderFilter filterNormalY_;
     robotUtils::FirstOrderFilter filterNormalZ_;
     robotUtils::FirstOrderFilter filterPositionX_;
     robotUtils::FirstOrderFilter filterPositionY_;
     robotUtils::FirstOrderFilter filterPositionZ_;

     double filterNormalTimeConstant_;
     double filterPositionTimeConstant_;
     double filterNormalGain_;
     double filterPositionGain_;

     loco::Vector normalInWorldFrameFilterInput_;
     loco::Position positionInWorldFrameFilterInput_;
     loco::Vector normalInWorldFrameFilterOutput_;
     loco::Position positionInWorldFrameFilterOutput_;
     //---

     /*! Estimate the free plane parameters and update the terrain model.
      */
     void updatePlaneEstimation();

     /*! Update last foot positions and torso pose for a leg.
      * @param[in,out] leg The leg to measure.
      */
     void updateLocalMeasuresOfLeg(const loco::LegBase& leg);

  }; // class
} /* namespace loco */


#endif /* LOCO_TERRAINPERCEPTIONFREEPLANE_HPP_ */
