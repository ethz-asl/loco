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
/*!
 * @file     LocomotionControllerJump.cpp
 * @author   wko
 * @date     Jun, 2014
 * @version  1.0
 * @ingroup
 * @brief
 */
#include "loco/locomotion_controller/LocomotionControllerJump.hpp"
#include "starlethModel/RobotModel.hpp"
namespace loco {

LocomotionControllerJump::LocomotionControllerJump(
    LegGroup* legs, TorsoBase* torso, TerrainPerceptionBase* terrainPerception,
    ContactDetectorBase* contactDetector, LimbCoordinatorBase* limbCoordinator,
    FootPlacementStrategyBase* footPlacementStrategy,
    TorsoControlBase* baseController,
    VirtualModelController* virtualModelController,
    ContactForceDistributionBase* contactForceDistribution,
    ParameterSet* parameterSet,
    TerrainModelBase* terrainModel)
    : LocomotionControllerBase(),
      runtime_(0.0),
      isInitialized_(false),
      legs_(legs),
      torso_(torso),
      terrainPerception_(terrainPerception),
      contactDetector_(contactDetector),
      limbCoordinator_(limbCoordinator),
      footPlacementStrategy_(footPlacementStrategy),
      torsoController_(baseController),
      virtualModelController_(virtualModelController),
      contactForceDistribution_(contactForceDistribution),
      parameterSet_(parameterSet),
      eventDetector_(new loco::EventDetector),
      terrainModel_(terrainModel)
{

}

LocomotionControllerJump::~LocomotionControllerJump() {
  delete eventDetector_;
}

bool LocomotionControllerJump::initialize(double dt) {
  isInitialized_ = false;
  runtime_ = 0.0;

  for (auto leg : *legs_) {
    if (!leg->initialize(dt)) {
      return false;
    }
  }
  if (!torso_->initialize(dt)) {
    return false;
  }

  TiXmlHandle hLoco(
      parameterSet_->getHandle().FirstChild("LocomotionController"));

  if (!terrainModel_->initialize(dt)) {
    return false;
  }

  if (!terrainPerception_->initialize(dt)) {
    return false;
  }

  if (!contactDetector_->initialize(dt)) {
    return false;
  }
  if (!limbCoordinator_->initialize(dt)) {
    return false;
  }
  if (!footPlacementStrategy_->loadParameters(hLoco)) {
    return false;
  }
  if (!footPlacementStrategy_->initialize(dt)) {
    return false;
  }

  if (!trajectoryFollower_.loadTrajectory(hLoco)) {
    return false;
  }

  if (!trajectoryFollower_.initialize(dt)) {
    return false;
  }

  if (!torsoController_->loadParameters(hLoco)) {
    return false;
  }
  if (!torsoController_->initialize(dt)) {
    return false;
  }

  if (!contactForceDistribution_->loadParameters(hLoco)) {
    return false;
  }

  if (!virtualModelController_->loadParameters(hLoco)) {
    return false;
  }

  isInitialized_ = true;
  return isInitialized_;
}
bool LocomotionControllerJump::advanceMeasurements(double dt) {
  if (!isInitialized_) {
    return false;
  }

  //--- Update sensor measurements.
  for (auto leg : *legs_) {
    if (!leg->advance(dt)) {
      return false;
    }
  }

  if(!torso_->advance(dt)) {
    return false;
  }

  if (!contactDetector_->advance(dt)) {
    return false;
  }
  //---
  return true;
}

bool LocomotionControllerJump::advanceSetPoints(double dt) {
  if (!isInitialized_) {
    return false;
  }

  //--- Update timing.

  int iLeg =0;
  for (auto leg : *legs_) {
  //--- defined by the "planning" / timing
    //-- NEEDS TO BE SET FOR TERRAINPERCEPTION
//    leg->setShouldBeGrounded()


//    leg->setShouldBeGrounded(gaitPattern_->shouldBeLegGrounded(iLeg));
//    leg->setStanceDuration(gaitPattern_->getStanceDuration(iLeg));
//    leg->setSwingDuration(gaitPattern_->getStrideDuration()-gaitPattern_->getStanceDuration(iLeg));
//    //---
//
//    //--- timing measurements
//    leg->setPreviousSwingPhase(leg->getSwingPhase());
//    leg->setSwingPhase(gaitPattern_->getSwingPhaseForLeg(iLeg));
//    leg->setPreviousStancePhase(leg->getStancePhase());
//    leg->setStancePhase(gaitPattern_->getStancePhaseForLeg(iLeg));
    //---


    iLeg++;
  }


  //---

  /* Update legs state using the event detection */
  if (!eventDetector_->advance(dt, *legs_)) {
    return false;
  }

  //--- Update knowledge about environment
  if (!terrainPerception_->advance(dt)) {
    return false;
  }
  //---

  /* Decide if a leg is a supporting one */
  if (!limbCoordinator_->advance(dt)) {
    return false;
  }

  /* Set the position or torque reference */
  if(!footPlacementStrategy_->advance(dt)) {
    return false;
  }
  if (!torsoController_->advance(dt)) {
    return false;
  }
  if(!virtualModelController_->compute()) {
    return false;
  }

  runtime_ += dt;
  return true;
}



TorsoBase* LocomotionControllerJump::getTorso() {
  return torso_;
}
LegGroup* LocomotionControllerJump::getLegs() {
  return legs_;
}

FootPlacementStrategyBase* LocomotionControllerJump::getFootPlacementStrategy() {
  return footPlacementStrategy_;
}

TorsoControlBase* LocomotionControllerJump::getTorsoController() {
  return torsoController_;
}

VirtualModelController* LocomotionControllerJump::getVirtualModelController() {
  return virtualModelController_;
}

ContactForceDistributionBase* LocomotionControllerJump::getContactForceDistribution() {
  return contactForceDistribution_;
}

LimbCoordinatorBase* LocomotionControllerJump::getLimbCoordinator() {
  return limbCoordinator_;
}

TerrainPerceptionBase* LocomotionControllerJump::getTerrainPerception() {
  return terrainPerception_;
}

bool LocomotionControllerJump::isInitialized() const {
  return isInitialized_;
}

double LocomotionControllerJump::getRuntime() const {
  return runtime_;
}

} /* namespace loco */

