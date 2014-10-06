/*!
* @file     LimbCoordinatorDynamicGait.cpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#include "loco/limb_coordinator/LimbCoordinatorDynamicGait.hpp"
#include "RobotModel_common.hpp"

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

	  //--- Set control mode
    if (leg->isSupportLeg()) {
      desiredJointControlModes.setConstant(robotModel::AM_Torque);
    }
    else {
      desiredJointControlModes.setConstant(robotModel::AM_Position);
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
