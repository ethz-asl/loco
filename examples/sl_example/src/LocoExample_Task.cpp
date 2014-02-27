/*
 * LocoExample_Task.cpp
 *
 *  Created on: Feb 27, 2014
 *      Author: gech
 */

#include "LocoExample_Task.hpp"
#include "TerrainPlane.hpp"

#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"
#include "loco/gait_pattern/GaitPatternAPS.hpp"
#include "loco/limb_coordinator/LimbCoordinatorDynamicGait.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyInvertedPendulum.hpp"
#include "loco/torso_control/TorsoControlDynamicGait.hpp"

#include "loco/common/LegStarlETH.hpp"
#include "loco/common/TorsoStarlETH.hpp"
namespace robotTask {

LocoExample::LocoExample(robotModel::RobotModel* robotModel):
    TaskRobotBase("LocoExample", robotModel)
{


}

LocoExample::~LocoExample() {

}

bool LocoExample::LocoExample::add()
{
  double dt = time_step_;
  static robotTerrain::TerrainPlane terrain;
  static loco::LegGroup legs;
  static loco::LegStarlETH leftForeLeg("leftFore", 0, robotModel_);
  static loco::LegStarlETH rightForeLeg("rightFore", 1, robotModel_);
  static loco::LegStarlETH leftHindLeg("leftHind", 2, robotModel_);
  static loco::LegStarlETH rightHindLeg("rightHind", 3, robotModel_);
  legs.addLeg(&leftForeLeg);
  legs.addLeg(&rightForeLeg);
  legs.addLeg(&leftHindLeg);
  legs.addLeg(&rightHindLeg);

  static loco::TorsoStarlETH torso;


  static loco::APS aps(0.8, 0.8, 0.5, 0.5, 0.5, 0.5, 0.5);
  static loco::GaitPatternAPS gaitPatternAPS;
  gaitPatternAPS.initialize(aps, dt);

  static loco::LimbCoordinatorDynamicGait limbCoordinator(&legs, &torso, &gaitPatternAPS);
  static loco::FootPlacementStrategyInvertedPendulum footPlacementStrategy(&legs, &torso, robotModel_, &terrain);
  static loco::TorsoControlDynamicGait baseControl(&legs, &torso, &terrain);

  static loco::LocomotionControllerDynamicGait locomotionController(&legs, &torso, robotModel_, &terrain, &limbCoordinator, &footPlacementStrategy, &baseControl);

  locomotionController_ = &locomotionController;
  return true;
}

bool LocoExample::init()
{

  return true;
}

bool LocoExample::run()
{
  locomotionController_->advance(time_step_);
  return true;
}

bool LocoExample::change()
{
  return true;
}

} /* namespace robotTask */
