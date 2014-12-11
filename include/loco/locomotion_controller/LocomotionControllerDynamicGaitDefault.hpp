/*****************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Christian Gehring, Péter Fankhauser, C. Dario Bellicoso, Stelian Coros
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*
 * LocomotionControllerDynamicGaitDefault.hpp
 *
 *  Created on: Aug 22, 2014
 *      Author: gech
 */

#ifndef LOCO_LOCOMOTIONCONTROLLERDYNAMICGAITDEFAULT_HPP_
#define LOCO_LOCOMOTIONCONTROLLERDYNAMICGAITDEFAULT_HPP_

#include "loco/locomotion_controller/LocomotionControllerBase.hpp"
#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"

#include "loco/common/TerrainModelBase.hpp"

#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"
#include "loco/gait_pattern/GaitPatternAPS.hpp"
#include "loco/gait_pattern/GaitPatternFlightPhases.hpp"
#include "loco/limb_coordinator/LimbCoordinatorDynamicGait.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyInvertedPendulum.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyFreePlane.hpp"
#include "loco/torso_control/TorsoControlDynamicGait.hpp"
#include "loco/torso_control/TorsoControlDynamicGaitFreePlane.hpp"
#include "loco/motion_control/VirtualModelController.hpp"
#include "loco/contact_force_distribution/ContactForceDistribution.hpp"
#include "loco/contact_detection/ContactDetectorBase.hpp"

#include "loco/mission_control/MissionControlBase.hpp"
#include "loco/mission_control/MissionControlSpeedFilter.hpp"

#include "loco/tools/TuningWithSliderboardLocoDynamicGait.hpp"


#include "loco/common/LegStarlETH.hpp"
#include "loco/common/TorsoStarlETH.hpp"
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/ParameterSet.hpp"
#include <memory>
#include <string>

#include "robotUtils/terrains/TerrainBase.hpp"

namespace loco {

class LocomotionControllerDynamicGaitDefault: public LocomotionControllerBase {
 public:
  LocomotionControllerDynamicGaitDefault(const std::string& parameterFile,
                                         robotModel::RobotModel* robotModel,
                                         robotTerrain::TerrainBase* terrain,
                                         double dt);
  virtual ~LocomotionControllerDynamicGaitDefault();

  /*!
   * Initializes locomotion controller
   * @param dt the time step [s]
   * @return true if successfull.
   */
  virtual bool initialize(double dt);

  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual bool advanceMeasurements(double dt);
  virtual bool advanceSetPoints(double dt);
  virtual bool isInitialized() const;

  void setDesiredBaseTwistInHeadingFrame(const Twist& desiredBaseTwistInHeadingFrame);

  LocomotionControllerDynamicGait* getLocomotionControllerDynamicGait();
  const LocomotionControllerDynamicGait& getLocomotionControllerDynamicGait() const;
  const MissionControlSpeedFilter& getMissionController() const;
  MissionControlSpeedFilter& getMissionController();
  ParameterSet* getParameterSet();

  const std::string& getGaitName() const;
  void setGaitName(const std::string& name);

  double getStrideDuration() const;
  double getStridePhase() const;
  bool setToInterpolated(const LocomotionControllerDynamicGaitDefault& controller1,  const LocomotionControllerDynamicGaitDefault& controller2, double t);
  GaitPatternFlightPhases* getGaitPattern();
  TorsoBase* getTorso();
  LegGroup* getLegs();
  ContactForceDistributionBase* getContactForceDistribution();
  const Twist& getDesiredBaseTwistInHeadingFrame() const;

  /*! @returns the run time of the controller in seconds.
   */
  virtual double getRuntime() const;
 private:
  robotModel::RobotModel* robotModel_;
  std::shared_ptr<ParameterSet> parameterSet_;
  std::shared_ptr<LegGroup> legs_;
  std::shared_ptr<LegStarlETH> leftForeLeg_;
  std::shared_ptr<LegStarlETH> rightForeLeg_;
  std::shared_ptr<LegStarlETH> leftHindLeg_;
  std::shared_ptr<LegStarlETH> rightHindLeg_;
  std::shared_ptr<TorsoStarlETH> torso_;
  std::shared_ptr<TerrainModelBase> terrainModel_;
  std::shared_ptr<TerrainPerceptionBase> terrainPerception_;
  std::shared_ptr<GaitPatternAPS> gaitPatternAPS_;
  std::shared_ptr<GaitPatternFlightPhases> gaitPatternFlightPhases_;
  std::shared_ptr<LimbCoordinatorDynamicGait> limbCoordinator_;
  std::shared_ptr<FootPlacementStrategyBase> footPlacementStrategy_;
  std::shared_ptr<TorsoControlDynamicGaitFreePlane> torsoController_;
  std::shared_ptr<ContactForceDistribution> contactForceDistribution_;
  std::shared_ptr<VirtualModelController> virtualModelController_;
  std::shared_ptr<ContactDetectorBase> contactDetector_;
  std::shared_ptr<MissionControlSpeedFilter> missionController_;
  std::shared_ptr<LocomotionControllerDynamicGait> locomotionController_;
};

} /* namespace loco */

#endif /* LOCOMOTIONCONTROLLERDYNAMICGAITDEFAULT_HPP_ */
