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
  footPlacementStrategy_.reset(new loco::FootPlacementStrategyInvertedPendulum(legs_.get(), torso_.get(), &terrain_));
  baseControl_.reset(new loco::TorsoControlDynamicGait (legs_.get(), torso_.get(), &terrain_));

  contactForceDistribution_.reset(new loco::ContactForceDistribution(robotModel_));
  contactForceDistribution_->setTerrain(&terrain_);
  virtualModelController_.reset(new loco::VirtualModelController(robotModel_, *contactForceDistribution_.get()));

  locomotionController_.reset(new loco::LocomotionControllerDynamicGait(legs_.get(), torso_.get(), robotModel_, &terrain_, limbCoordinator_.get(), footPlacementStrategy_.get(), baseControl_.get(), virtualModelController_.get(), contactForceDistribution_.get()));


  return true;
}

bool LocoExample::init()
{
  locomotionController_->initialize(time_step_);
  return true;
}

bool LocoExample::run()
{
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
