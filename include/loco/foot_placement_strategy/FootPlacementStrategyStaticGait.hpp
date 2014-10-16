/*
 * FootPlacementStrategyStaticGait.hpp
 *
 *  Created on: Oct 6, 2014
 *      Author: dario
 */

#ifndef LOCO_FOOTPLACEMENTSTRATEGYSTATICGAIT_HPP_
#define LOCO_FOOTPLACEMENTSTRATEGYSTATICGAIT_HPP_


#include "loco/foot_placement_strategy/FootPlacementStrategyFreePlane.hpp"

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlStaticGait.hpp"

#include "loco/common/TorsoBase.hpp"
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/LegGroup.hpp"

namespace loco {

class FootPlacementStrategyStaticGait: public FootPlacementStrategyFreePlane {
 public:
  FootPlacementStrategyStaticGait(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain);
  virtual ~FootPlacementStrategyStaticGait();

  virtual bool advance(double dt);
  virtual bool initialize(double dt);

  virtual Position getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep);
  virtual Position getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(const LegBase& leg);
  virtual Position getPositionFootAtLiftOffToDesiredFootHoldInControlFrame(const LegBase& leg);

  virtual Position getPositionDesiredFootHoldOrientationOffsetInWorldFrame(const LegBase& leg, const Position& positionWorldToDesiredFootHoldBeforeOrientationOffsetInWorldFrame);

  virtual Position getPositionWorldToValidatedDesiredFootHoldInWorldFrame(int legId) const;

  virtual void setCoMControl(CoMOverSupportPolygonControlBase* comControl);

 protected:

  bool footHoldPlanned_;

  CoMOverSupportPolygonControlStaticGait* comControl_;

  Position positionWorldToCenterOfFeetAtLiftOffInWorldFrame_;
  std::vector<Position> positionBaseOnTerrainToDefaultFootInControlFrame_;
  std::vector<Position> positionWorldToValidatedDesiredFootHoldInWorldFrame_;

  std::vector<Position> newFootHolds_;

  virtual void setFootTrajectory(LegBase* leg);
  virtual void regainContact(LegBase* leg, double dt);

  virtual Position generateFootHold(LegBase* leg);

  virtual Position getValidatedFootHold(const Position& positionWorldToDesiredFootHoldInWorldFrame);

};

} /* namespace loco */


#endif /* LOCO_FOOTPLACEMENTSTRATEGYSTATICGAIT_HPP_ */
