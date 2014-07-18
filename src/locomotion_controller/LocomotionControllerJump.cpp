/*!
 * @file     LocomotionControllerJump.cpp
 * @author   wko
 * @date     Jun, 2014
 * @version  1.0
 * @ingroup
 * @brief
 */
#include "loco/locomotion_controller/LocomotionControllerJump.hpp"
#include "RobotModel.hpp"
namespace loco {

LocomotionControllerJump::LocomotionControllerJump(
    LegGroup* legs, TorsoBase* torso, TerrainPerceptionBase* terrainPerception,
    ContactDetectorBase* contactDetector, LimbCoordinatorBase* limbCoordinator,
    FootPlacementStrategyBase* footPlacementStrategy,
    MotorVelocityController* motorVelocityController,
    TorsoControlBase* baseController,
    VirtualModelController* virtualModelController,
    ContactForceDistributionBase* contactForceDistribution,
    ParameterSet* parameterSet)
    : LocomotionControllerBase(),
      isInitialized_(false),
      legs_(legs),
      torso_(torso),
      terrainPerception_(terrainPerception),
      contactDetector_(contactDetector),
      limbCoordinator_(limbCoordinator),
      footPlacementStrategy_(footPlacementStrategy),
      motorVelocityController_(motorVelocityController),
      torsoController_(baseController),
      virtualModelController_(virtualModelController),
      contactForceDistribution_(contactForceDistribution),
      parameterSet_(parameterSet) {

}

LocomotionControllerJump::~LocomotionControllerJump() {

}

bool LocomotionControllerJump::initialize(double dt) {
  isInitialized_ = false;

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

  if (!motorVelocityController_->loadParameters(hLoco)) {
    return false;
  }

  if (!motorVelocityController_->initialize(dt)) {
    return false;
  }

  if (!trajectoryFollower_.inVelocityMode()) {
    motorVelocityController_->setInVelocityMode(false);
    dynamic_cast<TorsoControlJump*>(torsoController_)->setInTorsoPositionMode(
        true);
    dynamic_cast<TorsoControlJump*>(torsoController_)->setTrajectoryFollower(
        trajectoryFollower_);
  } else {
    motorVelocityController_->setInVelocityMode(true);
    dynamic_cast<TorsoControlJump*>(torsoController_)->setInTorsoPositionMode(
        false);
    motorVelocityController_->setTrajectoryFollower(trajectoryFollower_);
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

bool LocomotionControllerJump::advance(double dt) {
  if (!isInitialized_) {
    return false;
  }

  for (auto leg : *legs_) {
    leg->advance(dt);
  }

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
    if (leg->wasInStanceMode() && leg->isInSwingMode()) {
      // possible lift-off
      leg->getStateLiftOff()->setFootPositionInWorldFrame(
          leg->getWorldToFootPositionInWorldFrame());  // or base2foot?
      leg->getStateLiftOff()->setHipPositionInWorldFrame(
          leg->getWorldToHipPositionInWorldFrame());
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

  motorVelocityController_->advance(dt);

  torsoController_->advance(dt);

  footPlacementStrategy_->advance(dt);

  if (!virtualModelController_->compute()) {
    return false;
  }

  /* Set desired joint positions, torques and control mode */
  LegBase::JointControlModes desiredJointControlModes;
  int iLeg = 0;
  for (auto leg : *legs_) {
    if (leg->isAndShouldBeGrounded() && !leg->getStateTouchDown()->isNow()) {
      if (!trajectoryFollower_.inVelocityMode()) {
        desiredJointControlModes.setConstant(robotModel::AM_Torque);
      } else {
        desiredJointControlModes.setConstant(robotModel::AM_Velocity);
      }
    } else {
      desiredJointControlModes.setConstant(robotModel::AM_Position);
    }
    leg->setDesiredJointControlModes(desiredJointControlModes);
    iLeg++;
  }

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

} /* namespace loco */

