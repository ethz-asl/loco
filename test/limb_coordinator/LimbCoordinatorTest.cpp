/*!
* @file     FootPlacementTest.cpp
* @author   Christian Gehring
* @date     Jun, 2013
* @version  1.0
* @ingroup
* @brief
*/


#include <gtest/gtest.h>


#include "loco/LimbCoordinatorDynamicGait.hpp"
#include "loco/GaitPatternAPS.hpp"

TEST(LimbCoordinatorTest, test) {
  loco::APS aps(0.8, 0.8, 0.5, 0.5, 0.5, 0.5, 0.5);
  loco::GaitPatternAPS gaitPatternAPS;
  double dt = 0.0025;
  gaitPatternAPS.initialize(aps, dt);
  loco::LimbCoordinatorDynamicGait limbCoordinator(&gaitPatternAPS);
}
