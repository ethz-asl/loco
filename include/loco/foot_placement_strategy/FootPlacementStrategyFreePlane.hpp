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

    virtual void advance(double dt);
    virtual bool initialize(double dt);

    Position positionWorldToHipOnPlaneAlongNormalInWorldFrame_[4];
    Position positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame_[4];
    Position positionDesiredFootOnTerrainToDesiredFootInWorldFrame_[4];

    Position positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[4];
    Position positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[4];

    Position positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[4];

   protected:
    virtual Position getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep);
    virtual bool getBestFootholdsFromCurrentFootholdInWorldFrame(loco::Position& positionWorldToFootInWorldFrame);

    virtual Position getPositionProjectedOnPlaneAlongSurfaceNormal(const Position& position);
    virtual Position getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(const LegBase& leg);
    virtual Position getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(const LegBase& leg,
                                                                                const Position& positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame);
    virtual double getInterpolationPhase(const LegBase& leg);


    virtual Position getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame(const LegBase& leg);
    virtual Position getPositionDesiredFootHoldOnTerrainFeedBackInControlFrame(const LegBase& leg);

  };

} /* namespace loco */


#endif /* LOCO_FOOTPLACEMENTSTRATEGYFREEPLANE_HPP_ */
