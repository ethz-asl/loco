/*!
* @file     LocomotionControllerDynamicGait.cpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"
#include "Rotations.hpp"

namespace loco {

LocomotionControllerDynamicGait::LocomotionControllerDynamicGait(LegGroup* legs, TorsoBase* torso, robotModel::RobotModel* robotModel,
                                                                 robotTerrain::TerrainBase* terrain, LimbCoordinatorBase* limbCoordinator,
                                                                 FootPlacementStrategyBase* footPlacementStrategy, TorsoControlBase* baseController,
                                                                 VirtualModelController* virtualModelController, ContactForceDistributionBase* contactForceDistribution,
                                                                 ParameterSet* parameterSet) :
    LocomotionControllerBase(),
    isInitialized_(false),
    legs_(legs),
    torso_(torso),
    robotModel_(robotModel),
    terrain_(terrain),
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
    //  std::cout << *leg << std::endl;
//    std::cout << "leg: " << leg->getName() << (leg->isGrounded() ? "is grounded" : "is NOT grounded") << std::endl;
  }
  torso_->advance(dt);
  limbCoordinator_->advance(dt);

  for (auto leg : *legs_) {
    const double swingPhase = leg->getSwingPhase();
    if ((swingPhase >= 0 && swingPhase <= 0.5) && leg->isInStanceMode()) {
      // possible lift-off
      leg->getStateLiftOff()->setFootPositionInWorldFrame(leg->getWorldToFootPositionInWorldFrame()); // or base2foot?
      leg->getStateLiftOff()->setHipPositionInWorldFrame(leg->getWorldToHipPositionInWorldFrame());
    }
  }
  footPlacementStrategy_->advance(dt);
  torsoController_->advance(dt);
  if(!virtualModelController_->compute()) {
    std::cout << "Error from virtual model controller" << std::endl;
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


bool LocomotionControllerDynamicGait::isInitialized() const {
  return isInitialized_;
}

} /* namespace loco */

