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

#include "loco/common/TerrainModelBase.hpp"

#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"
#include "loco/gait_pattern/GaitPatternAPS.hpp"
#include "loco/gait_pattern/GaitPatternFlightPhases.hpp"
#include "loco/limb_coordinator/LimbCoordinatorDynamicGait.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyInvertedPendulum.hpp"
#include "loco/torso_control/TorsoControlDynamicGait.hpp"
#include "loco/motion_control/VirtualModelController.hpp"
#include "loco/contact_force_distribution/ContactForceDistribution.hpp"
#include "loco/contact_detection/ContactDetectorBase.hpp"

#include "loco/mission_control/MissionControlJoystick.hpp"
#include "loco/tools/TuningWithSliderboardLocoDynamicGait.hpp"


#include "loco/common/LegStarlETH.hpp"
#include "loco/common/TorsoStarlETH.hpp"
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/ParameterSet.hpp"

#include "RobotModel.hpp"
#include "robotUtils/terrains/TerrainPlane.hpp"
#include "loco/common/TerrainModelHorizontalPlane.hpp"

#include "OoqpEigenInterface.hpp"


#include "loco/common/TerrainModelHorizontalPlane.hpp"
#include "loco/terrain_perception/TerrainPerceptionHorizontalPlane.hpp"
#include "loco/common/LegLinkGroup.hpp"

#include "loco/contact_detection/ContactDetectorConstantDuringStance.hpp"

TEST(LocomotionControllerTest, test) {
  double dt = 0.0025;



  robotTerrain::TerrainPlane terrain;
  robotModel::RobotModel robotModel;
  robotModel.init();
  robotModel.update();
  robotModel::RobotModel* robotModel_ = &robotModel;

  std::shared_ptr<loco::LegGroup> legs_;
  std::shared_ptr<loco::LegStarlETH> leftForeLeg_;
  std::shared_ptr<loco::LegStarlETH> rightForeLeg_;
  std::shared_ptr<loco::LegStarlETH> leftHindLeg_;
  std::shared_ptr<loco::LegStarlETH> rightHindLeg_;
  std::shared_ptr<loco::TorsoStarlETH> torso_;
  std::shared_ptr<loco::TerrainModelBase> terrainModel_;
  std::shared_ptr<loco::TerrainPerceptionBase> terrainPerception_;
  std::shared_ptr<loco::GaitPatternAPS> gaitPatternAPS_;
  std::shared_ptr<loco::LimbCoordinatorDynamicGait> limbCoordinator_;
  std::shared_ptr<loco::FootPlacementStrategyInvertedPendulum> footPlacementStrategy_;
  std::shared_ptr<loco::TorsoControlDynamicGait> torsoController_;
  std::shared_ptr<loco::ContactForceDistribution> contactForceDistribution_;
  std::shared_ptr<loco::VirtualModelController> virtualModelController_;
  std::shared_ptr<loco::LocomotionControllerDynamicGait> locomotionController_;
  std::shared_ptr<loco::ParameterSet> parameterSet_;
  std::shared_ptr<loco::ContactDetectorBase> contactDetector_;


  /* create terrain */
   terrainModel_.reset(new loco::TerrainModelHorizontalPlane);


   /* create legs */
   leftForeLeg_.reset(new loco::LegStarlETH("leftFore", 0,  robotModel_));
   rightForeLeg_.reset(new loco::LegStarlETH("rightFore", 1,  robotModel_));
   leftHindLeg_.reset(new loco::LegStarlETH("leftHind", 2,  robotModel_));
   rightHindLeg_.reset(new loco::LegStarlETH("rightHind", 3,  robotModel_));


   legs_.reset( new loco::LegGroup(leftForeLeg_.get(), rightForeLeg_.get(), leftHindLeg_.get(), rightHindLeg_.get()));

   /* create torso */
   torso_.reset(new loco::TorsoStarlETH(robotModel_));
   terrainPerception_.reset(new loco::TerrainPerceptionHorizontalPlane((loco::TerrainModelHorizontalPlane*)terrainModel_.get(), legs_.get()));

   /* create locomotion controller */
   contactDetector_.reset(new loco::ContactDetectorConstantDuringStance(legs_.get()));
   gaitPatternAPS_.reset(new loco::GaitPatternAPS);
   limbCoordinator_.reset(new loco::LimbCoordinatorDynamicGait(legs_.get(), torso_.get(), gaitPatternAPS_.get()));
   footPlacementStrategy_.reset(new loco::FootPlacementStrategyInvertedPendulum(legs_.get(), torso_.get(), terrainModel_.get()));
   torsoController_.reset(new loco::TorsoControlDynamicGait (legs_.get(), torso_.get(), terrainModel_.get()));
   contactForceDistribution_.reset(new loco::ContactForceDistribution(torso_, legs_, terrainModel_));
   virtualModelController_.reset(new loco::VirtualModelController(legs_, torso_, contactForceDistribution_));
   locomotionController_.reset(new loco::LocomotionControllerDynamicGait(legs_.get(), torso_.get(), terrainPerception_.get(), contactDetector_.get(), limbCoordinator_.get(), footPlacementStrategy_.get(), torsoController_.get(), virtualModelController_.get(), contactForceDistribution_.get(), parameterSet_.get()));
   locomotionController_->advance(dt);

}
