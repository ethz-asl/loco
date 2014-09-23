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
                                                                 ParameterSet* parameterSet) :
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
    timeSinceTorqueControl_(0.0),
    timeIntervalToSwitchToPositionControl_(0.05)
{
  eventDetector_ = new loco::EventDetector;
}

LocomotionControllerDynamicGait::LocomotionControllerDynamicGait() :
    LocomotionControllerBase(),
  isInitialized_(false),
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
  eventDetector_(nullptr)
{

}

LocomotionControllerDynamicGait::~LocomotionControllerDynamicGait() {

}

bool LocomotionControllerDynamicGait::initialize(double dt)
{
  isInitialized_ = false;

  for (auto leg : *legs_) {
    if(!leg->initialize(dt)) { return false; }
  }

  if (!torso_->initialize(dt)) { return false; }
  TiXmlHandle hLoco(parameterSet_->getHandle().FirstChild("LocomotionController"));

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

  runtime_ = 0.0;
  
  timeSinceTorqueControl_ = 0.0;
  timeIntervalToSwitchToPositionControl_ = 0.0;

  isInitialized_ = true;
  return isInitialized_;
}


bool LocomotionControllerDynamicGait::advance(double dt) {
  if (!isInitialized_) { return false; }

  for (auto leg : *legs_) { leg->advance(dt); }

  torso_->advance(dt);

  if (!contactDetector_->advance(dt)) { return false; }
  if (!terrainPerception_->advance(dt)) { return false; }

  limbCoordinator_->advance(dt);

  /* Update legs state using the event detection */
  eventDetector_->advance(dt, *legs_);

  /* Decide wether the legs should be torque o position controlled. Set desired joint positions, torques and control mode
   * State change policy:
   *  1. If leg is a swing leg, it should be position controlled.
   *  2. If leg is a stance leg, it should be torque controlled.
   *  3. As soon as a touchdown is detected, control should switch to torque control.
   *  4. If control switches to torque control, it should not switch back to position for at least a given time interval.
   *  5. A leg switches to stance mode when it is grounded. It switches to swing mode after a certain time interval.
   */
  LegBase::JointControlModes desiredJointControlModes;
  int iLeg = 0;

  /*
  //--- Old policy
  for (auto leg : *legs_) {
    if (leg->isAndShouldBeGrounded()) {
      desiredJointControlModes.setConstant(robotModel::AM_Torque);
    } else {
      desiredJointControlModes.setConstant(robotModel::AM_Position);
    }
    leg->setDesiredJointControlModes(desiredJointControlModes);
    iLeg++;
  }
  //---
  */

  //--- Policy working in simulation
  for (auto leg : *legs_) {
    if (leg->getDesiredJointControlModes().isConstant(robotModel::AM_Torque)) {
      timeSinceTorqueControl_ += dt;
      std::cout << "leg: " << leg->getId() << " time: " << timeSinceTorqueControl_ << std::endl;
    }

    if (leg->isGrounded()) {
      desiredJointControlModes.setConstant(robotModel::AM_Torque);
    }

    if (!leg->isAndShouldBeGrounded() && timeSinceTorqueControl_>= timeIntervalToSwitchToPositionControl_) {
      desiredJointControlModes.setConstant(robotModel::AM_Position);
      timeSinceTorqueControl_ = 0.0;
    }

    leg->setDesiredJointControlModes(desiredJointControlModes);
    iLeg++;
  }
  //---

  /*
  LegBase::JointControlModes desiredJointControlModes;
  int iLeg = 0;
  for (auto leg : *legs_) {

    // 2.
    if (leg->isGrounded() || timeSinceTorqueControl_ < timeIntervalToSwitchToPositionControl_) {
      desiredJointControlModes.setConstant(robotModel::AM_Torque);
      timeSinceTorqueControl_ += dt;
    }

    // 1.
    if (!leg->shouldBeGrounded() && timeSinceTorqueControl_ >= timeIntervalToSwitchToPositionControl_) {
      timeSinceTorqueControl_ = 0.0;
      desiredJointControlModes.setConstant(robotModel::AM_Position);
    }

    leg->setDesiredJointControlModes(desiredJointControlModes);
    iLeg++;
  }
  */

  /* Set the position or torque reference */
  footPlacementStrategy_->advance(dt);
  torsoController_->advance(dt);
  if(!virtualModelController_->compute()) { return false; }

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

