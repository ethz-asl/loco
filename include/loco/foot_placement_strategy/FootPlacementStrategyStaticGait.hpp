/*
 * FootPlacementStrategyStaticGait.hpp
 *
 *  Created on: Oct 6, 2014
 *      Author: dario
 */

#ifndef LOCO_FOOTPLACEMENTSTRATEGYSTATICGAIT_HPP_
#define LOCO_FOOTPLACEMENTSTRATEGYSTATICGAIT_HPP_


#include "loco/foot_placement_strategy/FootPlacementStrategyFreePlane.hpp"
#include "loco/common/TorsoBase.hpp"
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/LegGroup.hpp"

namespace loco {

class FootPlacementStrategyStaticGait: public FootPlacementStrategyFreePlane {
 public:
  FootPlacementStrategyStaticGait(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain);
  virtual ~FootPlacementStrategyStaticGait();

  virtual Position getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep);
  virtual Position getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(const LegBase& leg);
  virtual Position getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame(const LegBase& leg);

  virtual Position getPositionDesiredFootHoldOrientationOffsetInControlFrame(const LegBase& leg);

  virtual void validateFootHold(Position& positionWorldToDesiredFootHoldInWorldFrame);

 protected:

};

} /* namespace loco */


#endif /* LOCO_FOOTPLACEMENTSTRATEGYSTATICGAIT_HPP_ */
