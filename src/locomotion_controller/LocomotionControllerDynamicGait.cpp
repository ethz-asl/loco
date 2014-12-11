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
* @file     LocomotionControllerDynamicGait.cpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"
//#include "Rotations.hpp"
#include "starlethModel/RobotModel.hpp"
namespace loco {

LocomotionControllerDynamicGait::LocomotionControllerDynamicGait(LegGroup* legs, TorsoBase* torso,
                                                                 TerrainPerceptionBase* terrainPerception,
                                                                 ContactDetectorBase* contactDetector,
                                                                 LimbCoordinatorBase* limbCoordinator,
                                                                 FootPlacementStrategyBase* footPlacementStrategy, TorsoControlBase* baseController,
                                                                 VirtualModelController* virtualModelController, ContactForceDistributionBase* contactForceDistribution,
                                                                 ParameterSet* parameterSet,
                                                                 GaitPatternBase* gaitPattern,
                                                                 TerrainModelBase* terrainModel) :
    LocomotionControllerBase(),
    isInitialized_(false),
    runtime_(0.0),
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
    gaitPattern_(gaitPattern),
    terrainModel_(terrainModel)
{

}

LocomotionControllerDynamicGait::LocomotionControllerDynamicGait() :
    LocomotionControllerBase(),
    isInitialized_(false),
    runtime_(0.0),
    legs_(nullptr),
    torso_(nullptr),
    terrainPerception_(nullptr),
    contactDetector_(nullptr),
    limbCoordinator_(nullptr),
    footPlacementStrategy_(nullptr),
    torsoController_(nullptr),
    virtualModelController_(nullptr),
    contactForceDistribution_(nullptr),
    parameterSet_(nullptr),
    eventDetector_(nullptr),
    gaitPattern_(nullptr),
    terrainModel_(nullptr)
{

}

LocomotionControllerDynamicGait::~LocomotionControllerDynamicGait() {
  delete eventDetector_;
}

bool LocomotionControllerDynamicGait::initialize(double dt)
{
  isInitialized_ = false;

  for (auto leg : *legs_) {
    if(!leg->initialize(dt)) { return false; }
  }

  if (!torso_->initialize(dt)) { return false; }
  TiXmlHandle hLoco(parameterSet_->getHandle().FirstChild("LocomotionController"));

  if (!terrainModel_->initialize(dt)) {
    return false;
  }

  if (!terrainPerception_->initialize(dt)) {
    return false;
  }

  if (!contactDetector_->initialize(dt)) {
    return false;
  }

  if (!eventDetector_->initialize(dt)) {
    return false;
  }

  if (!limbCoordinator_->loadParameters(hLoco)) {
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


  if (!gaitPattern_->loadParameters(TiXmlHandle(hLoco.FirstChild("LimbCoordination")))) {
	return false;
  }
  if(!gaitPattern_->initialize(dt)) {
	return false;
  }

  runtime_ = 0.0;
  
  isInitialized_ = true;
  return isInitialized_;
}


bool LocomotionControllerDynamicGait::advanceMeasurements(double dt) {
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
bool LocomotionControllerDynamicGait::advanceSetPoints(double dt) {

  //--- Update timing.
  gaitPattern_->advance(dt);

//  bool standing = true;
//  for (auto leg: *legs_) {
//    standing &= leg->isInStandConfiguration();
//  }
//  if (standing) {
//    gaitPattern_->setStridePhase(0.0);
//  }

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


TorsoBase* LocomotionControllerDynamicGait::getTorso() {
  return torso_;
}


LegGroup* LocomotionControllerDynamicGait::getLegs() {
  return legs_;
}


FootPlacementStrategyBase* LocomotionControllerDynamicGait::getFootPlacementStrategy() {
  return footPlacementStrategy_;
}


const FootPlacementStrategyBase& LocomotionControllerDynamicGait::getFootPlacementStrategy() const {
  return *footPlacementStrategy_;
}


VirtualModelController* LocomotionControllerDynamicGait::getVirtualModelController() {
  return virtualModelController_;
}


ContactForceDistributionBase* LocomotionControllerDynamicGait::getContactForceDistribution() {
  return contactForceDistribution_;
}


LimbCoordinatorBase*  LocomotionControllerDynamicGait::getLimbCoordinator() {
  return limbCoordinator_;
}


const LimbCoordinatorBase& LocomotionControllerDynamicGait::getLimbCoordinator() const {
  return *limbCoordinator_;
}


const TorsoControlBase& LocomotionControllerDynamicGait::getTorsoController() const {
  return *torsoController_;
}

TorsoControlBase& LocomotionControllerDynamicGait::getTorsoController()  {
  return *torsoController_;
}

const VirtualModelController& LocomotionControllerDynamicGait::getVirtualModelController() const {
  return *virtualModelController_;
}


TerrainPerceptionBase* LocomotionControllerDynamicGait::getTerrainPerception() {
  return terrainPerception_;
}


bool LocomotionControllerDynamicGait::isInitialized() const {
  return isInitialized_;
}



double LocomotionControllerDynamicGait::getRuntime() const {
  return runtime_;
}

bool LocomotionControllerDynamicGait::setToInterpolated(const LocomotionControllerDynamicGait& controller1, const LocomotionControllerDynamicGait& controller2, double t) {
  if (!limbCoordinator_->setToInterpolated(controller1.getLimbCoordinator(), controller2.getLimbCoordinator(), t)) {
    return false;
  }
  if (!torsoController_->setToInterpolated(controller1.getTorsoController(), controller2.getTorsoController(), t)) {
    return false;
  }

  if (!footPlacementStrategy_->setToInterpolated(controller1.getFootPlacementStrategy(), controller2.getFootPlacementStrategy(), t)) {
     return false;
   }

  if (!virtualModelController_->setToInterpolated(controller1.getVirtualModelController(), controller2.getVirtualModelController(), t)) {
    return false;
  }

  return true;
}

} /* namespace loco */

