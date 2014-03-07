/*
 * LocoExample_Task.hpp
 *
 *  Created on: Feb 27, 2014
 *      Author: gech
 */

#ifndef LOCOEXAMPLE_TASK_HPP_
#define LOCOEXAMPLE_TASK_HPP_

#include "TaskRobotBase.hpp"
#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"

#include "TerrainPlane.hpp"

#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"
#include "loco/gait_pattern/GaitPatternAPS.hpp"
#include "loco/limb_coordinator/LimbCoordinatorDynamicGait.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyInvertedPendulum.hpp"
#include "loco/torso_control/TorsoControlDynamicGait.hpp"
#include "loco/motion_control/VirtualModelController.hpp"
#include "loco/contact_force_distribution/ContactForceDistribution.hpp"

#include "loco/mission_control/MissionControlJoystick.hpp"

#include "loco/common/LegStarlETH.hpp"
#include "loco/common/TorsoStarlETH.hpp"
#include "loco/common/ParameterSet.hpp"
#include <memory>

namespace robotTask {

class LocoExample: public robotTask::TaskRobotBase {
 public:
  LocoExample(robotModel::RobotModel* robotModel);
  virtual ~LocoExample();

  /*! Adds the task.
   * @return  true if successful
   */
  virtual bool add();

  /*! Initializes the task.
   * @return  true if successful
   */
  virtual bool init();

  /*! Runs the task.
   * @return  true if successful
   */
  virtual bool run();

  /*! Changes the parameters of the task.
   * Loads a menu where the user can change parameters of this task.
   * @return  true if successful
   */
  virtual bool change();

  loco::LocomotionControllerDynamicGait*  getLocomotionController();

 public:
  std::shared_ptr<loco::LegGroup> legs_;
  std::shared_ptr<loco::LegStarlETH> leftForeLeg_;
  std::shared_ptr<loco::LegStarlETH> rightForeLeg_;
  std::shared_ptr<loco::LegStarlETH> leftHindLeg_;
  std::shared_ptr<loco::LegStarlETH> rightHindLeg_;
  std::shared_ptr<loco::TorsoStarlETH> torso_;

  robotTerrain::TerrainPlane terrain_;

  std::shared_ptr<loco::GaitPatternAPS> gaitPatternAPS_;
  std::shared_ptr<loco::LimbCoordinatorDynamicGait> limbCoordinator_;
  std::shared_ptr<loco::FootPlacementStrategyInvertedPendulum> footPlacementStrategy_;
  std::shared_ptr<loco::TorsoControlDynamicGait> torsoController_;
  std::shared_ptr<loco::ContactForceDistribution> contactForceDistribution_;
  std::shared_ptr<loco::VirtualModelController> virtualModelController_;
  std::shared_ptr<loco::LocomotionControllerDynamicGait> locomotionController_;
  std::shared_ptr<loco::ParameterSet> parameterSet_;

  std::shared_ptr<loco::MissionControlJoystick> missionController_;
};

} /* namespace robotTask */

#endif /* LOCOEXAMPLE_TASK_HPP_ */
