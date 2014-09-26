/*
 * FootPlacementStrategyFreePlane.cpp
 *
 *  Created on: Sep 16, 2014
 *      Author: dario
 */

#include "loco/foot_placement_strategy/FootPlacementStrategyFreePlane.hpp"
#include "loco/common/TorsoBase.hpp"
#include "loco/temp_helpers/math.hpp"

namespace loco {

  FootPlacementStrategyFreePlane::FootPlacementStrategyFreePlane(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain) :
      FootPlacementStrategyInvertedPendulum(legs, torso, terrain)
  {
  }


  FootPlacementStrategyFreePlane::~FootPlacementStrategyFreePlane() {

  }


  void FootPlacementStrategyFreePlane::advance(double dt) {

    for (auto leg : *legs_) {
      if (!leg->isSupportLeg()) {
        Position positionWorldToFootInWorldFrame = getDesiredWorldToFootPositionInWorldFrame(leg, 0.0);

        if (!getBestFootholdsFromCurrentFootholdInWorldFrame(positionWorldToFootInWorldFrame)) {
          // TODO: handle exception
        }

        leg->setDesireWorldToFootPositionInWorldFrame(positionWorldToFootInWorldFrame); // for debugging
        const Position positionBaseToFootInWorldFrame = positionWorldToFootInWorldFrame - torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
        const Position positionBaseToFootInBaseFrame  = torso_->getMeasuredState().getOrientationWorldToBase().rotate(positionBaseToFootInWorldFrame);
        leg->setDesiredJointPositions(leg->getJointPositionsFromBaseToFootPositionInBaseFrame(positionBaseToFootInBaseFrame));
      }
    }
  }


  bool FootPlacementStrategyFreePlane::getBestFootholdsFromCurrentFootholdInWorldFrame(loco::Position& positionWorldToFootInWorldFrame) {
    return true;
  }


} /* namespace loco */
