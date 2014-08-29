/*
 * LocomotionControllerDynamicGaitDefault.cpp
 *
 *  Created on: Aug 22, 2014
 *      Author: gech
 */

#include "loco/locomotion_controller/LocomotionControllerDynamicGaitDefault.hpp"
#include "loco/common/TerrainModelHorizontalPlane.hpp"
#include "loco/terrain_perception/TerrainPerceptionHorizontalPlane.hpp"
#include "loco/common/LegLinkGroup.hpp"

#include "loco/contact_detection/ContactDetectorConstantDuringStance.hpp"
#include "loco/mission_control/MissionControlSpeedFilter.hpp"

namespace loco {

LocomotionControllerDynamicGaitDefault::LocomotionControllerDynamicGaitDefault(const std::string& parameterFile,
                                                                               robotModel::RobotModel* robotModel,
                                                                               robotTerrain::TerrainBase* terrain,
                                                                               double dt): LocomotionControllerBase(), robotModel_(robotModel)
{


    parameterSet_.reset(new loco::ParameterSet());
    if (!parameterSet_->loadXmlDocument(parameterFile)) {
      std::cout << "Could not load parameter file: " << parameterFile  << std::endl;
      return;
    }
    // print parameter file for debugging
  //  parameterSet_->getHandle().ToNode()->ToDocument()->Print();


    /* create terrain */
    terrainModel_.reset(new loco::TerrainModelHorizontalPlane);


    /* create legs */
    leftForeLeg_.reset(new loco::LegStarlETH("leftFore", 0,  robotModel));
    rightForeLeg_.reset(new loco::LegStarlETH("rightFore", 1,  robotModel));
    leftHindLeg_.reset(new loco::LegStarlETH("leftHind", 2,  robotModel));
    rightHindLeg_.reset(new loco::LegStarlETH("rightHind", 3,  robotModel));


    legs_.reset( new loco::LegGroup(leftForeLeg_.get(), rightForeLeg_.get(), leftHindLeg_.get(), rightHindLeg_.get()));

    /* create torso */
    torso_.reset(new loco::TorsoStarlETH(robotModel));
    terrainPerception_.reset(new loco::TerrainPerceptionHorizontalPlane((loco::TerrainModelHorizontalPlane*)terrainModel_.get(), legs_.get()));

    /* create locomotion controller */
    contactDetector_.reset(new loco::ContactDetectorConstantDuringStance(legs_.get()));
    gaitPatternAPS_.reset(new loco::GaitPatternAPS);
    gaitPatternFlightPhases_.reset(new loco::GaitPatternFlightPhases);
    limbCoordinator_.reset(new loco::LimbCoordinatorDynamicGait(legs_.get(), torso_.get(), gaitPatternFlightPhases_.get()));
    footPlacementStrategy_.reset(new loco::FootPlacementStrategyInvertedPendulum(legs_.get(), torso_.get(), terrainModel_.get()));
    torsoController_.reset(new loco::TorsoControlDynamicGait (legs_.get(), torso_.get(), terrainModel_.get()));
    contactForceDistribution_.reset(new loco::ContactForceDistribution(torso_, legs_, terrainModel_));
    virtualModelController_.reset(new loco::VirtualModelController(legs_, torso_, contactForceDistribution_));

    missionController_.reset(new loco::MissionControlSpeedFilter);

    locomotionController_.reset(new loco::LocomotionControllerDynamicGait(legs_.get(), torso_.get(), terrainPerception_.get(), contactDetector_.get(), limbCoordinator_.get(), footPlacementStrategy_.get(), torsoController_.get(), virtualModelController_.get(), contactForceDistribution_.get(), parameterSet_.get()));


}

LocomotionControllerDynamicGaitDefault::~LocomotionControllerDynamicGaitDefault() {

}

bool LocomotionControllerDynamicGaitDefault::initialize(double dt) {

  if (!missionController_->loadParameters(parameterSet_->getHandle())) {
    std::cout << "Could not parameters for mission controller: " << std::endl;
    return false;
  }

  if (!missionController_->initialize(dt)) {
    return false;
  }

  if (!locomotionController_->initialize(dt)) {
    return false;
  }
  return true;
}

bool LocomotionControllerDynamicGaitDefault::advance(double dt) {

  if (!missionController_->advance(dt)) {
    return false;
  }
  torso_->setDesiredBaseTwistInHeadingFrame(missionController_->getDesiredBaseTwistInHeadingFrame());
  torsoController_->setDesiredPositionOffetInWorldFrame(missionController_->getDesiredPositionOffsetInWorldFrame());
  torsoController_->setDesiredOrientationOffset(missionController_->getDesiredOrientationOffset());


  if (!locomotionController_->advance(dt)) {
    return false;
  }
//  std::cout << "Torso: \n";
//  std::cout << *torso_ << std::endl;


//  std::cout << "Actuation: \n";
  /* copy desired commands from locomotion controller to robot model */
  int iLeg = 0;
  for (auto leg : *legs_) {
    robotModel_->act().setPosOfLeg(leg->getDesiredJointPositions() ,iLeg);
    robotModel_->act().setTauOfLeg(leg->getDesiredJointTorques() ,iLeg);
//    std::cout << "Torques: " << leg->getDesiredJointTorques() << std::endl;
    robotModel::VectorActMLeg modes = leg->getDesiredJointControlModes().matrix();
    robotModel_->act().setModeOfLeg(modes,iLeg);
//    std::cout << *leg << std::endl;
    iLeg++;
  }

  return true;
}


LocomotionControllerDynamicGait* LocomotionControllerDynamicGaitDefault::getLocomotionControllerDynamicGait() {
  return locomotionController_.get();
}
const LocomotionControllerDynamicGait& LocomotionControllerDynamicGaitDefault::getLocomotionControllerDynamicGait() const {
  return *locomotionController_.get();
}

bool LocomotionControllerDynamicGaitDefault::isInitialized() const {
  return locomotionController_->isInitialized();
}

ParameterSet* LocomotionControllerDynamicGaitDefault::getParameterSet() {
  return parameterSet_.get();
}
void LocomotionControllerDynamicGaitDefault::setDesiredBaseTwistInHeadingFrame(const Twist& desiredBaseTwistInHeadingFrame) {
  missionController_->setUnfilteredDesiredBaseTwistInHeadingFrame(desiredBaseTwistInHeadingFrame);
}

const std::string& LocomotionControllerDynamicGaitDefault::getGaitName() const {
  return limbCoordinator_->getGaitPattern()->getName();
}
void LocomotionControllerDynamicGaitDefault::setGaitName(const std::string& name) {
  limbCoordinator_->getGaitPattern()->setName(name);
}

bool LocomotionControllerDynamicGaitDefault::setToInterpolated(const LocomotionControllerDynamicGaitDefault& controller1,  const LocomotionControllerDynamicGaitDefault& controller2, double t) {
  if(!missionController_->setToInterpolated(controller1.getMissionController(),
                                            controller2.getMissionController(),
                                            t)) {
    return false;
  }

  if(!locomotionController_->setToInterpolated(controller1.getLocomotionControllerDynamicGait(),
                                               controller2.getLocomotionControllerDynamicGait(),
                                               t)) {
    return false;
  }
  return true;
}

double LocomotionControllerDynamicGaitDefault::getStrideDuration() const {
  return limbCoordinator_->getGaitPattern()->getStrideDuration();
}

double LocomotionControllerDynamicGaitDefault::getStridePhase() const {
  return torso_->getStridePhase();
}

GaitPatternFlightPhases* LocomotionControllerDynamicGaitDefault::getGaitPattern() {
  return gaitPatternFlightPhases_.get();
}

TorsoBase* LocomotionControllerDynamicGaitDefault::getTorso() {
  return torso_.get();
}
LegGroup* LocomotionControllerDynamicGaitDefault::getLegs() {
  return legs_.get();
}

ContactForceDistributionBase* LocomotionControllerDynamicGaitDefault::getContactForceDistribution() {
  return contactForceDistribution_.get();
}

const MissionControlSpeedFilter& LocomotionControllerDynamicGaitDefault::getMissionController() const {
  return *missionController_.get();
}

MissionControlSpeedFilter& LocomotionControllerDynamicGaitDefault::getMissionController() {
  return *missionController_.get();
}

const Twist& LocomotionControllerDynamicGaitDefault::getDesiredBaseTwistInHeadingFrame() const {
  return missionController_->getDesiredBaseTwistInHeadingFrame();
}

} /* namespace loco */
