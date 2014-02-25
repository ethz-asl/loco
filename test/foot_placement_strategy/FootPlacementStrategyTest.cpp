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
#include "RobotModel.hpp"
#include "TerrainPlane.hpp"

TEST(FootPlacementTest, test) {
  robotTerrain::TerrainPlane terrain;

  double dt = 0.0025;
  robotModel::RobotModel robotModel;
  loco::APS aps(0.8, 0.8, 0.5, 0.5, 0.5, 0.5, 0.5);
  loco::GaitPatternAPS gaitPatternAPS;

  gaitPatternAPS.initialize(aps, dt);
  loco::LimbCoordinatorDynamicGait limbCoordinator(&gaitPatternAPS);
  loco::FootPlacementStrategyInvertedPendulum ip(&robotModel, &terrain, &limbCoordinator);
}
