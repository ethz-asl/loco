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
#include "loco/torso_control/TorsoControlDynamicGait.hpp"

#include "loco/common/LegStarlETH.hpp"
#include "loco/common/TorsoStarlETH.hpp"

#include "loco/common/ParameterSet.hpp"

#include "RobotModel.hpp"
#include "TerrainPlane.hpp"


TEST(LocomotionControllerTest, test) {
  double dt = 0.0025;


  robotTerrain::TerrainPlane terrain;
  robotModel::RobotModel robotModel;
  robotModel.init();
  robotModel.update();


  loco::ParameterSet parameterSet;
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


  loco::APS aps(0.8, 0.8, 0.5, 0.5, 0.5, 0.5, 0.5);
  loco::GaitPatternAPS gaitPatternAPS;
  gaitPatternAPS.initialize(aps, dt);

  loco::LimbCoordinatorDynamicGait limbCoordinator(&legs, &torso, &gaitPatternAPS);
  loco::FootPlacementStrategyInvertedPendulum footPlacementStrategy(&legs, &torso, &terrain);
  loco::TorsoControlDynamicGait baseControl(&legs, &torso, &terrain);

   loco::ContactForceDistribution contactForceDistribution_(&robotModel);
   contactForceDistribution_.setTerrain(&terrain);
   loco::VirtualModelController virtualModelController_(&robotModel, contactForceDistribution_);

   loco::LocomotionControllerDynamicGait locomotionController(&legs, &torso, &robotModel, &terrain, &limbCoordinator, &footPlacementStrategy, &baseControl, &virtualModelController_, &contactForceDistribution_, &parameterSet);

   locomotionController.advance(dt);

}
