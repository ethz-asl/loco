/*!
* @file     BaseControlTest.cpp
* @author   Christian Gehring
* @date     Jun, 2013
* @version  1.0
* @ingroup
* @brief
*/


#include <gtest/gtest.h>

#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"
#include "loco/gait_pattern/GaitPatternAPS.hpp"
#include "loco/limb_coordinator/LimbCoordinatorDynamicGait.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyInvertedPendulum.hpp"
#include "loco/base_control/BaseControlDynamicGait.hpp"
#include "RobotModel.hpp"

TEST(LocomotionControllerTest, test) {
  double dt = 0.0025;

  robotModel::RobotModel robotModel;

  loco::APS aps(0.8, 0.8, 0.5, 0.5, 0.5, 0.5, 0.5);
  loco::GaitPatternAPS gaitPatternAPS;

  gaitPatternAPS.initialize(aps, dt);
  loco::LimbCoordinatorDynamicGait limbCoordinator(&gaitPatternAPS);

  loco::FootPlacementStrategyInvertedPendulum footPlacementStrategy(&robotModel, &limbCoordinator);
  loco::BaseControlDynamicGait baseControl;

  loco::LocomotionControllerDynamicGait locomotionController(&robotModel, &limbCoordinator, &footPlacementStrategy, &baseControl);
  locomotionController.advance(dt);

}
