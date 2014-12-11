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
* @file     LimbCoordinatorDynamicGait.cpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#include "loco/limb_coordinator/LimbCoordinatorDynamicGait.hpp"
#include "starlethModel/RobotModel_common.hpp"

#include "loco/state_switcher/StateSwitcher.hpp"

namespace loco {


LimbCoordinatorDynamicGait::LimbCoordinatorDynamicGait(LegGroup* legs, TorsoBase* torso, GaitPatternBase* gaitPattern, bool isUpdatingStridePhase) :
    LimbCoordinatorBase(),
    isUpdatingStridePhase_(isUpdatingStridePhase),
    legs_(legs),
    torso_(torso),
    gaitPattern_(gaitPattern)
{
  // initialize state for each leg
	for (auto leg: *legs_) {
    state_[leg->getId()] = -1;
	}

}


LimbCoordinatorDynamicGait::~LimbCoordinatorDynamicGait() {

}


void LimbCoordinatorDynamicGait::setIsUpdatingStridePhase(bool isUpdatingStridePhase) {
  isUpdatingStridePhase_ = isUpdatingStridePhase;
}


bool LimbCoordinatorDynamicGait::isUpdatingStridePhase() const {
  return isUpdatingStridePhase_;
}


bool LimbCoordinatorDynamicGait::initialize(double dt) {
	for (int i=0; i<4; i++) {
		state_[i] = -1;
	}
  if(!advance(0.0)) {
    return false;
  }
  return true;
}


bool LimbCoordinatorDynamicGait::advance(double dt) {
  int iLeg = 0;
  LegBase::JointControlModes desiredJointControlModes;

  /* state_
   * 0: stance phase normal condition
   * 1: swing phase normal condition
   * 2: stance, but slipping
   * 3: stance, but lost contact / not yet touch-down
   * 4: swing, but late lift-off
   * 5: late swing, but early touch-down
   * 6: middle swing, but bumped into obstacle while swinging
   */

  StateSwitcher* stateSwitcher;

  for (auto leg : *legs_) {

    stateSwitcher = leg->getStateSwitcher();

    //leg->setWasInStanceMode(leg->isInStanceMode());
    //leg->setIsInStanceMode(isLegInStanceMode(iLeg));

	//--- Decide if a leg is supporting or not
//    if ( leg->getStateTouchDown()->isNow() ) {
//    	leg->setIsSupportLeg(true);
//    }
//
	  // Check timing
	  if (leg->shouldBeGrounded()) {
		  // stance mode according to plan
		  if (leg->isGrounded()) {
			  if (leg->isSlipping()) {
				  // not safe to use this leg as support leg
				  leg->setIsSupportLeg(false);
//				  state_[iLeg] = 2;

				  stateSwitcher->setState(StateSwitcher::States::StanceSlipping);

				  // todo think harder about this
			  }
			  else {
				  // safe to use this leg as support leg
				  leg->setIsSupportLeg(true);
//				  state_[iLeg] = 0;

				  stateSwitcher->setState(StateSwitcher::States::StanceNormal);

			  }
		  }
		  else {
			  // not yet touch-down
			  // lost contact
			  leg->setIsSupportLeg(false);
//			  state_[iLeg] = 3;
			  stateSwitcher->setState(StateSwitcher::States::StanceLostContact);

		  }
	  }
	  else {
		  // swing mode according to plan
		  if (leg->isGrounded()) {

			  if (leg->getSwingPhase() <= 0.3) {
				  // leg should lift-off (late lift-off)
				  leg->setIsSupportLeg(false);
//				  state_[iLeg] = 4;
				  stateSwitcher->setState(StateSwitcher::States::SwingLateLiftOff);
			  }
			  else if (leg->getSwingPhase() > 0.6) {
				  // early touch-down
				  leg->setIsSupportLeg(true);
//				  state_[iLeg] = 5;
				  stateSwitcher->setState(StateSwitcher::States::SwingEarlyTouchDown);
			  }
			  else {
				  // leg bumped into obstacle
				  leg->setIsSupportLeg(false); // true
//				  state_[iLeg] = 6;
				  stateSwitcher->setState(StateSwitcher::States::SwingBumpedIntoObstacle);
			  }
		  }
		  else {
			  // leg is on track
			  leg->setIsSupportLeg(false);
//			  state_[iLeg] = 1;
			  stateSwitcher->setState(StateSwitcher::States::SwingNormal);
		  }
	  }
    //---

	  // Override support leg flag
	  if (leg->isInStandConfiguration()) leg->setIsSupportLeg(true);

	  //--- Set control mode
    if (leg->isSupportLeg()) {
      desiredJointControlModes.setConstant(robotModel::AM_Torque);
      leg->setDesiredJointPositions(leg->getMeasuredJointPositions());
    }
    else {
      desiredJointControlModes.setConstant(robotModel::AM_Position);
      leg->setDesiredJointTorques(leg->getMeasuredJointTorques());
    }
    leg->setDesiredJointControlModes(desiredJointControlModes);

    //---

    iLeg++;
  }

  return true;
}


void LimbCoordinatorDynamicGait::setStridePhase(double stridePhase)  {
  gaitPattern_->setStridePhase(stridePhase);
}


bool LimbCoordinatorDynamicGait::isLegInStanceMode(int iLeg) {
  const double swingPhase = gaitPattern_->getSwingPhaseForLeg(iLeg);
  if (swingPhase > 0.5 && legs_->getLeg(iLeg)->isGrounded()) return true; // early touch-down -> switch to stance mode
  return (swingPhase < 0 || swingPhase > 1); // stance mode
}


GaitPatternBase* LimbCoordinatorDynamicGait::getGaitPattern() {
  return gaitPattern_;
}


const GaitPatternBase& LimbCoordinatorDynamicGait::getGaitPattern() const {
  return *gaitPattern_;
}


bool LimbCoordinatorDynamicGait::loadParameters(const TiXmlHandle& handle)
{
  return true;
}


bool LimbCoordinatorDynamicGait::setToInterpolated(const LimbCoordinatorBase& limbCoordinator1, const LimbCoordinatorBase& limbCoordinator2, double t) {
  const LimbCoordinatorDynamicGait& coordinator1 = static_cast<const LimbCoordinatorDynamicGait&>(limbCoordinator1);
  const LimbCoordinatorDynamicGait& coordinator2 = static_cast<const LimbCoordinatorDynamicGait&>(limbCoordinator2);
  if (!gaitPattern_->setToInterpolated(coordinator1.getGaitPattern(), coordinator2.getGaitPattern(), t)) {
    return false;
  }
  return true;
}


} /* namespace loco */
