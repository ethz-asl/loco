/*!
* @file     FootPlacementTest.cpp
* @author   Christian Gehring
* @date     Jun, 2013
* @version  1.0
* @ingroup
* @brief
*/


#include <gtest/gtest.h>


#include "loco/limb_coordinator/LimbCoordinatorDynamicGait.hpp"
#include "loco/gait_pattern/GaitPatternAPS.hpp"

#include "loco/common/TorsoStarlETH.hpp"

TEST(LimbCoordinatorTest, test) {
  loco::APS aps(0.8, 0.8, 0.5, 0.5, 0.5, 0.5, 0.5);
  loco::GaitPatternAPS gaitPatternAPS;
  double dt = 0.0025;

  loco::LegGroup legs;
  loco::TorsoStarlETH torso;

  gaitPatternAPS.initialize(aps, dt);
  loco::LimbCoordinatorDynamicGait limbCoordinator(&legs, &torso, &gaitPatternAPS);
}
