/*
 * FootPlacementStrategyFreePlane.hpp
 *
 *  Created on: Sep 16, 2014
 *      Author: dario
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
#include "robotUtils/loggers/Logger.hpp"
#include "loco/common/TerrainModelBase.hpp"

#include "BoundedRBF1D.hpp"

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
