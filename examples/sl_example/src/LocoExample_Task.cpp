/*
 * LocoExample_Task.cpp
 *
 *  Created on: Feb 27, 2014
 *      Author: gech
 */

#include "LocoExample_Task.hpp"

namespace robotTask {

LocoExample::LocoExample(robotModel::RobotModel* robotModel):
    TaskRobotBase("LocoExample", robotModel)
{


}

LocoExample::~LocoExample() {

}


bool LocoExample::LocoExample::add()
{
  double dt = time_step_;
  std::string gait = "StaticLateralWalk";
//  std::string gait = "WalkingTrot";

  std::cout << "Gait: " << gait << std::endl;
  std::string parameterFile = std::string(getenv("LAB_ROOT")) +"/locomotionControl/examples/sl_example/parameters/" + gait + "Sim.xml";

  parameterSet_.reset(new loco::ParameterSet());

  if (!parameterSet_->loadXmlDocument(parameterFile)) {
    std::cout << "Could not load parameter file: " << parameterFile  << std::endl;
    return false;
  }


//  parameterSet_->getHandle().ToNode()->ToDocument()->Print();


  legs_.reset( new loco::LegGroup);
  leftForeLeg_.reset(new loco::LegStarlETH("leftFore", 0, robotModel_));
  rightForeLeg_.reset(new loco::LegStarlETH("rightFore", 1, robotModel_));
  leftHindLeg_.reset(new loco::LegStarlETH("leftHind", 2, robotModel_));
  rightHindLeg_.reset(new loco::LegStarlETH("rightHind", 3, robotModel_));
  legs_->addLeg(leftForeLeg_.get());
  legs_->addLeg(rightForeLeg_.get());
  legs_->addLeg(leftHindLeg_.get());
  legs_->addLeg(rightHindLeg_.get());

  torso_.reset(new loco::TorsoStarlETH(robotModel_));


  static loco::APS aps(0.8, 0.8, 0.5, 0.5, 0.5, 0.5, 0.5);
  gaitPatternAPS_.reset(new loco::GaitPatternAPS);
  gaitPatternAPS_->initialize(aps, dt);

  limbCoordinator_.reset(new loco::LimbCoordinatorDynamicGait(legs_.get(), torso_.get(), gaitPatternAPS_.get()));
  footPlacementStrategy_.reset(new loco::FootPlacementStrategyInvertedPendulum(legs_.get(), torso_.get(), terrain_.get()));
  torsoController_.reset(new loco::TorsoControlDynamicGait (legs_.get(), torso_.get(), terrain_.get()));

  contactForceDistribution_.reset(new loco::ContactForceDistribution(legs_, terrain_));
  virtualModelController_.reset(new loco::VirtualModelController(legs_, torso_, contactForceDistribution_));

  missionController_.reset(new loco::MissionControlJoystick(robotModel_));

  locomotionController_.reset(new loco::LocomotionControllerDynamicGait(legs_.get(), torso_.get(), robotModel_, terrain_.get(), limbCoordinator_.get(), footPlacementStrategy_.get(), torsoController_.get(), virtualModelController_.get(), contactForceDistribution_.get(), parameterSet_.get()));


  return true;
}

bool LocoExample::init()
{
  if (!missionController_->loadParameters(parameterSet_->getHandle())) {
    std::cout << "Could not parameters for mission controller: " << std::endl;
    return false;
  }


  if (!missionController_->initialize(time_step_)) {
    std::cout << "Could not initialize mission controller!" << std::endl;
  }

  if (!locomotionController_->initialize(time_step_)) {
    std::cout << "Could not initialize locomotion controller!" << std::endl;
    return false;
  }
  return true;
}

bool LocoExample::run()
{
  missionController_->advance(time_step_);
  torso_->getDesiredState().setBaseTwistInBaseFrame(missionController_->getDesiredBaseTwistInBaseFrame());
  locomotionController_->advance(time_step_);

  robotModel::VectorActMLeg legMode;
  int iLeg = 0;
  for (auto leg : *legs_) {
    robotModel_->act().setPosOfLeg(leg->getDesiredJointPositions() ,iLeg);
    robotModel_->act().setTauOfLeg(leg->getDesiredJointTorques() ,iLeg);
    robotModel::VectorActMLeg modes = leg->getDesiredJointControlModes().matrix();
    robotModel_->act().setModeOfLeg( modes,iLeg);

    iLeg++;
  }
  return true;
}

bool LocoExample::change()
{
  return true;
}

loco::LocomotionControllerDynamicGait*  LocoExample::getLocomotionController() {
  return locomotionController_.get();
}

} /* namespace robotTask */
