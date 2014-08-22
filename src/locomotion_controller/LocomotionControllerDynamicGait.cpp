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
    legs_(legs),
    torso_(torso),
    terrainPerception_(terrainPerception),
    contactDetector_(contactDetector),
    limbCoordinator_(limbCoordinator),
    footPlacementStrategy_(footPlacementStrategy),
    torsoController_(baseController),
    virtualModelController_(virtualModelController),
    contactForceDistribution_(contactForceDistribution),
    parameterSet_(parameterSet)
{

}

LocomotionControllerDynamicGait::~LocomotionControllerDynamicGait() {

}

bool LocomotionControllerDynamicGait::initialize(double dt)
{
  isInitialized_ = false;

  for (auto leg : *legs_) {
    if(!leg->initialize(dt)) {
      return false;
    }
    //  std::cout << *leg << std::endl;
  }
  if (!torso_->initialize(dt)) {
    return false;
  }



  TiXmlHandle hLoco(parameterSet_->getHandle().FirstChild("LocomotionController"));

  if (!terrainPerception_->initialize(dt)) {
    return false;
  }

  if (!contactDetector_->initialize(dt)) {
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


  isInitialized_ = true;
  return isInitialized_;
}

bool LocomotionControllerDynamicGait::advance(double dt) {
  if (!isInitialized_) {
//    std::cout << "locomotion controller is not initialized!\n" << std::endl;
    return false;
  }


  for (auto leg : *legs_) {
    leg->advance(dt);
//      std::cout << *leg << std::endl;
//    std::cout << "leg: " << leg->getName() << (leg->isGrounded() ? "is grounded" : "is NOT grounded") << std::endl;
  }

//  std::cout << "Torso:\n";
//  std::cout << *torso_ << std::endl;

  torso_->advance(dt);

  if (!contactDetector_->advance(dt)) {
    return false;
  }

  if (!terrainPerception_->advance(dt)) {
    return false;
  }

  limbCoordinator_->advance(dt);

  for (auto leg : *legs_) {
    const double swingPhase = leg->getSwingPhase();
//    std::cout << *leg << std::endl;

//    if (leg->isInStanceMode() != leg->wasInStanceMode()) {
//      std::cout << leg->getName() << " -----------------------------------------------------------------\n";
//    }
//
//    if ((swingPhase >= 0.0 && swingPhase <= 0.5) && leg->wasInStanceMode()) {
    if (leg->wasInStanceMode() && leg->isInSwingMode()) {
      // possible lift-off
      leg->getStateLiftOff()->setFootPositionInWorldFrame(leg->getWorldToFootPositionInWorldFrame()); // or base2foot?
      leg->getStateLiftOff()->setHipPositionInWorldFrame(leg->getWorldToHipPositionInWorldFrame());
//      std::cout << leg->getName() << ": lift-off" << std::endl;
      leg->getStateLiftOff()->setIsNow(true);
    } else {
      leg->getStateLiftOff()->setIsNow(false);
    }

    if (leg->wasInSwingMode() && leg->isInStanceMode()) {
      // possible touch-down
      leg->getStateTouchDown()->setIsNow(true);
    } else {
      leg->getStateTouchDown()->setIsNow(false);
    }

  }
  footPlacementStrategy_->advance(dt);
  torsoController_->advance(dt);
  if(!virtualModelController_->compute()) {
//    std::cout << "Error from virtual model controller" << std::endl;
    return false;
  }


  /* Set desired joint positions, torques and control mode */
  LegBase::JointControlModes desiredJointControlModes;
  int iLeg = 0;
  for (auto leg : *legs_) {
    if (leg->isAndShouldBeGrounded()) {
      desiredJointControlModes.setConstant(robotModel::AM_Torque);
    } else {
      desiredJointControlModes.setConstant(robotModel::AM_Position);
    }
    leg->setDesiredJointControlModes(desiredJointControlModes);
    iLeg++;
  }

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

bool LocomotionControllerDynamicGait::setToInterpolated(const LocomotionControllerDynamicGait& controller1, const LocomotionControllerDynamicGait& controller2, double t) {
  if (!limbCoordinator_->setToInterpolated(controller1.getLimbCoordinator(), controller2.getLimbCoordinator() ,t)) {
    return false;
  }
  if (!torsoController_->setToInterpolated(controller1.getTorsoController(), controller2.getTorsoController() ,t)) {
    return false;
  }

  if (!footPlacementStrategy_->setToInterpolated(controller1.getFootPlacementStrategy(), controller2.getFootPlacementStrategy() ,t)) {
     return false;
   }

  if (!virtualModelController_->setToInterpolated(controller1.getVirtualModelController(), controller2.getVirtualModelController(), t)) {
    return false;
  }

  return true;
}

} /* namespace loco */

