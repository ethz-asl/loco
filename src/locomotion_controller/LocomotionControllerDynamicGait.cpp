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
#include "RobotModel.hpp"
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
  torso_->setStridePhase(gaitPattern_->getStridePhase());

  int iLeg =0;
  for (auto leg : *legs_) {
	//--- defined by the "planning" / timing
    leg->setShouldBeGrounded(gaitPattern_->shouldBeLegGrounded(iLeg));
    leg->setStanceDuration(gaitPattern_->getStanceDuration(iLeg));
    leg->setSwingDuration(gaitPattern_->getStrideDuration()-gaitPattern_->getStanceDuration(iLeg));
    //---

    //--- timing measurements
    leg->setPreviousSwingPhase(leg->getSwingPhase());
    leg->setSwingPhase(gaitPattern_->getSwingPhaseForLeg(iLeg));
    leg->setPreviousStancePhase(leg->getStancePhase());
    leg->setStancePhase(gaitPattern_->getStancePhaseForLeg(iLeg));
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

