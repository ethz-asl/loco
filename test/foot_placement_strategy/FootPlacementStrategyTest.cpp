/*!
* @file     FootPlacementTest.cpp
* @author   Christian Gehring
* @date     Jun, 2013
* @version  1.0
* @ingroup
* @brief
*/


#include <gtest/gtest.h>
#include "loco/foot_placement_strategy/FootPlacementStrategyInvertedPendulum.hpp"
#include "loco/limb_coordinator/LimbCoordinatorDynamicGait.hpp"
#include "loco/gait_pattern/GaitPatternAPS.hpp"

#include "loco/common/LegStarlETH.hpp"
#include "loco/common/TorsoStarlETH.hpp"

#include "RobotModel.hpp"
#include "TerrainPlane.hpp"

TEST(FootPlacementTest, test) {
  robotTerrain::TerrainPlane terrain;

  double dt = 0.0025;
  robotModel::RobotModel robotModel;
  loco::APS aps(0.8, 0.8, 0.5, 0.5, 0.5, 0.5, 0.5);
  loco::GaitPatternAPS gaitPatternAPS;
  loco::LegGroup legs;
  loco::LegStarlETH leftForeLeg("leftFore", 0, &robotModel);
  loco::LegStarlETH rightForeLeg("rightFore", 1, &robotModel);
  loco::LegStarlETH leftHindLeg("leftHind", 2, &robotModel);
  loco::LegStarlETH rightHindLeg("rightHind", 3, &robotModel);
  legs.addLeg(&leftForeLeg);
  legs.addLeg(&rightForeLeg);
  legs.addLeg(&leftHindLeg);
  legs.addLeg(&rightHindLeg);

  loco::TorsoStarlETH torso(&robotModel);

  gaitPatternAPS.initialize(aps, dt);
  loco::LimbCoordinatorDynamicGait limbCoordinator(&legs, &torso, &gaitPatternAPS);

  robotModel.init();
  robotModel.update();

  loco::FootPlacementStrategyInvertedPendulum ip(&legs, &torso, &terrain);
  ip.advance(dt);
}
