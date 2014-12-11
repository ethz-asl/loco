/*****************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Christian Gehring, C. Dario Bellicoso, Péter Fankhauser, Stelian Coros
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
/*!
* @file     FootPlacementStrategyFreePlane.hpp
* @author   C. Dario Bellicoso, Christian Gehring
* @date     Sep 16, 2014
* @brief
*/
#ifndef LOCO_FOOTPLACEMENTSTRATEGYFREEPLANE_HPP_
#define LOCO_FOOTPLACEMENTSTRATEGYFREEPLANE_HPP_


#include "loco/common/TypeDefs.hpp"
#include "loco/common/TorsoBase.hpp"
#include "loco/common/LegGroup.hpp"

#include "loco/foot_placement_strategy/FootPlacementStrategyInvertedPendulum.hpp"

#include "tinyxml.h"
#include <Eigen/Core>

#include "loco/temp_helpers/Trajectory.hpp"

#include "kindr/rotations/RotationEigen.hpp"
#include "loco/common/TerrainModelBase.hpp"

#include "robotUtils/function_approximators/polyharmonicSplines/BoundedRBF1D.hpp"

namespace loco {

  class FootPlacementStrategyFreePlane: public FootPlacementStrategyInvertedPendulum {

   public:
    FootPlacementStrategyFreePlane(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain);
    virtual ~FootPlacementStrategyFreePlane();

    virtual bool advance(double dt);
    virtual bool initialize(double dt);

    Position positionWorldToHipOnPlaneAlongNormalInWorldFrame_[4];
    Position positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame_[4];
    Position positionDesiredFootOnTerrainToDesiredFootInWorldFrame_[4];

    Position positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[4];
    Position positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[4];

    Position positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[4];

   protected:

    /*! Compute and return the current desired foot position in world frame.
     * @params[in] leg The leg relative to the desired foot.
     * @returns The desired foot position in world frame.
     */
    virtual Position getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep);


    /*! Project a point on a plane along the plane's normal.
     * @params[in] position The coordinates of the point that has to be projected.
     * @returns The coordinates of the point projected on the terrain along the surface normal.
     */
    virtual Position getPositionProjectedOnPlaneAlongSurfaceNormal(const Position& position);


    virtual Position getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(const LegBase& leg);

    /*! Return the height component of the desired foot position in control frame.
     * @params[in] leg The desired foot's leg.
     * @params[in] positionHipOnTerrainToDesiredFootOnTerrainInControlFrame Position of the projection of the hip according to the desired configuration (from telescopic to lever)
     * @returns The desired foot position on the terrain in control frame.
     */
    virtual Position getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(const LegBase& leg,
                                                                                const Position& positionHipOnTerrainToDesiredFootOnTerrainInControlFrame);

    /*! Return the interpolation phase relative to a given leg.
     * @params[in] leg The leg relative the to the interpolating foot.
     * @returns The interpolation phase.
     */
    virtual double getInterpolationPhase(const LegBase& leg);

    /*! Evaluate the feed forward component for the desired foot hold
     * @params[in] leg The leg relative the to the desired foot hold.
     * @returns The feed forward component.
     */
    virtual Position getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame(const LegBase& leg);

    /*! Evaluate the feedback (inverted pendulum) component for the desired foot hold
     * @params[in] leg The leg relative the to the desired foot hold.
     * @returns The feedback component.
     */
    virtual Position getPositionDesiredFootHoldOnTerrainFeedBackInControlFrame(const LegBase& leg);

    /*! Returns an offset that sets the position from where the interpolation for the desired foot hold should start (depens on the value of telescopicLeverConfiguration_).
     * @params[in] leg The leg relative to the selected configuration.
     * @returns The offset for the desired configuration.
     */
    virtual Position getPositionVerticalHeightOnTerrainToLeverTelescopicConfigurationInWorldFrame(const LegBase& leg);

    /*! Adjust the hind foothold
     * @params[in] leg The leg relative to the selected configuration.
     * @returns The offset for the desired configuration.
     */
    virtual Position getOffsetDesiredFootOnTerrainToCorrectedFootOnTerrainInControlFrame(const LegBase& leg);


    /*! 0: telescopic
     *  1: lever
     *  in (0,1): combination of both
     */
    double telescopicLeverConfiguration_;

  };

} /* namespace loco */


#endif /* LOCO_FOOTPLACEMENTSTRATEGYFREEPLANE_HPP_ */
