/*
 * LimbCoordinatorJump.cpp
 *
 *  Created on: Oct 6, 2014
 *      Author: gech
 */

#include "loco/limb_coordinator/LimbCoordinatorJump.hpp"
#include "RobotModel_common.hpp"

namespace loco {

LimbCoordinatorJump::LimbCoordinatorJump(LegGroup* legs, TorsoBase* torso) : LimbCoordinatorBase(),
    legs_(legs),
    torso_(torso)
{

}

LimbCoordinatorJump::~LimbCoordinatorJump()
{
}

bool LimbCoordinatorJump::initialize(double dt) {
  return true;
}

bool LimbCoordinatorJump::advance(double dt) {

  StateSwitcher* stateSwitcher;
  LegBase::JointControlModes desiredJointControlModes;

   for (auto leg : *legs_) {

     stateSwitcher = leg->getStateSwitcher();
     stateSwitcher->setState(StateSwitcher::States::StanceNormal);
     if (leg->isGrounded()) {
       leg->setIsSupportLeg(true);
     }

     //--- Set control mode
     if (leg->isSupportLeg()) {
       desiredJointControlModes.setConstant(robotModel::AM_Torque);
     }
     else {
       desiredJointControlModes.setConstant(robotModel::AM_Position);
     }
     leg->setDesiredJointControlModes(desiredJointControlModes);
     //---
   }
  return true;
}

bool LimbCoordinatorJump::loadParameters(const TiXmlHandle& handle) {
  return true;
}

GaitPatternBase* LimbCoordinatorJump::getGaitPattern() {
  return nullptr;
}

} /* namespace loco */
