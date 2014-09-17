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

    void advance(double dt);

   protected:
    virtual bool getBestFootholdsFromCurrentFootholdInWorldFrame(loco::Position& positionWorldToFootInWorldFrame);
  };

} /* namespace loco */


#endif /* LOCO_FOOTPLACEMENTSTRATEGYFREEPLANE_HPP_ */
