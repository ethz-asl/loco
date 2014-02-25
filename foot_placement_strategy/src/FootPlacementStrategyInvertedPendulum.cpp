/*!
* @file 	FootPlacementStrategyInvertedPendulum.cpp
* @author 	Christian Gehring, Stelian Coros
* @date		Sep 7, 2012
* @version 	1.0
* @ingroup 	robotTask
* @brief
*/

#include "loco/FootPlacementStrategyInvertedPendulum.hpp"

#include "math.hpp"

namespace loco {

FootPlacementStrategyInvertedPendulum::FootPlacementStrategyInvertedPendulum() {

	stepFeedbackScale_ = 1.2;
	stepInterpolationFunction.clear();
	stepInterpolationFunction.addKnot(0, 0);
	stepInterpolationFunction.addKnot(0.6, 1);

	gravity_ = 9.81;

	swingFootHeightTrajectory_.clear();
	swingFootHeightTrajectory_.addKnot(0, 0);
	swingFootHeightTrajectory_.addKnot(0.65, 0.09);
	swingFootHeightTrajectory_.addKnot(1.0, 0);


  for (int iLeg=0; iLeg < 4; iLeg++) {
    estimatedGroundHeightCSw_[iLeg] = 0.0;
    stanceDuration_[iLeg] = 0.8;
    swingPhase_[iLeg] = 0.0;
    footLocationAtLiftOffCSw_[iLeg]  = Eigen::Vector3d::Zero();
    rHip_CSw_[iLeg] = Eigen::Vector3d::Zero();
    vHip_CSw_[iLeg] = Eigen::Vector3d::Zero();

  }

  vBase_CSw_ =  Eigen::Vector3d::Zero();

  desiredHeadingSpeed_ = 0.0;

}

FootPlacementStrategyInvertedPendulum::~FootPlacementStrategyInvertedPendulum() {

}

void FootPlacementStrategyInvertedPendulum::setFootLocationAtLiftOff(int iLeg, const Eigen::Vector3d& footLocationAtLiftOffCSw)
{
  footLocationAtLiftOffCSw_[iLeg] = footLocationAtLiftOffCSw;
}


void FootPlacementStrategyInvertedPendulum::setGravity(double gravity)
{
  gravity_ = gravity;
}

void FootPlacementStrategyInvertedPendulum::setSwingFootHeightTrajectory(const Trajectory1D& swingFootHeightTrajectory)
{
  swingFootHeightTrajectory_ = swingFootHeightTrajectory;
}


void FootPlacementStrategyInvertedPendulum::setSwingPhase(int iLeg, double swingPhase) {
  swingPhase_[iLeg] = swingPhase;
}
void FootPlacementStrategyInvertedPendulum::setStanceDuration(int iLeg, double stanceDuration) {
  stanceDuration_[iLeg] = stanceDuration;
}

void FootPlacementStrategyInvertedPendulum::setGroundHeight(int iLeg, double groundHeightCSw) {
  estimatedGroundHeightCSw_[iLeg] = groundHeightCSw;
}

void FootPlacementStrategyInvertedPendulum::setHipPosition(int iLeg, const Eigen::Vector3d& rHip_CSw) {
  rHip_CSw_[iLeg] = rHip_CSw;
}
void FootPlacementStrategyInvertedPendulum::setHipVelocity(int iLeg, const Eigen::Vector3d& vHip_CSw) {
  vHip_CSw_[iLeg] = vHip_CSw;
}
void FootPlacementStrategyInvertedPendulum::setBaseVelocity(int iLeg, const Eigen::Vector3d& vBase_CSw) {
  vBase_CSw_ = vBase_CSw;
}

void FootPlacementStrategyInvertedPendulum::setRotationWorldToBase(const RotationQuaternion& p_BW) {
  p_BW_ = p_BW;
}

void FootPlacementStrategyInvertedPendulum::setSteppingOffsetToHip(int iLeg, const Eigen::Vector3d& steppingOffsetToHip_CSmb) {
  steppingOffsetToHip_CSmb_[iLeg] = steppingOffsetToHip_CSmb;
}

void FootPlacementStrategyInvertedPendulum::setDesiredHeadingSpeed(double desiredForwardSpeed) {
  desiredHeadingSpeed_ = desiredForwardSpeed;
}

void FootPlacementStrategyInvertedPendulum::setFeedbackScale(double scale) {
  stepFeedbackScale_ = scale;
}




Eigen::Vector3d FootPlacementStrategyInvertedPendulum::getFootPositionForSwingLegCSw(int iLeg, double dt)
{

	const double swingPhase = swingPhase_[iLeg];
  Eigen::Vector3d rFootHoldOffset_CSw_default = p_BW_.inverseRotate(steppingOffsetToHip_CSmb_[iLeg]);

	/* inverted pendulum stepping offset */
	Eigen::Vector3d rRef_CSw = rHip_CSw_[iLeg];
	Eigen::Vector3d vRef_CSw = (vHip_CSw_[iLeg]+vBase_CSw_)/2.0;
	double invertedPendulumHeight = std::max((rRef_CSw.z() - estimatedGroundHeightCSw_[iLeg]), 0.0);
	Eigen::Vector3d vBaseDes_CSmb = Eigen::Vector3d(desiredHeadingSpeed_, 0, 0);
	Eigen::Vector3d vError_CSw = vRef_CSw - p_BW_.inverseRotate(vBaseDes_CSmb); // do not use A_WB*vBaseDes_CSmb
	Eigen::Vector3d rFootHoldOffset_CSw_invertedPendulum = vError_CSw*sqrt(invertedPendulumHeight/gravity_);

	/* feedforward stepping offset */
	//we also need to add a desired-velocity dependent feed forward step length
	double netCOMDisplacementPerStride = desiredHeadingSpeed_ * stanceDuration_[iLeg];
	//this is relative to the hip location, hence the divide by 2.0
	double feedForwardStepLength = netCOMDisplacementPerStride / 2.0;

	Eigen::Vector3d rFootHoldOffset_CSw_feedforward = p_BW_.inverseRotate(Eigen::Vector3d(feedForwardStepLength, 0.0, 0.0));


	Eigen::Vector3d rFootHoldOffset_CSw_final = rFootHoldOffset_CSw_default + rFootHoldOffset_CSw_feedforward + stepFeedbackScale_*rFootHoldOffset_CSw_invertedPendulum;
	rFootHoldOffset_CSw_final(2) = 0.0;
	Eigen::Vector3d rFootOffset_CSw = getCurrentFootPositionFromPredictedFootHoldLocation(std::min(swingPhase + dt, 1.0),  footLocationAtLiftOffCSw_[iLeg], rFootHoldOffset_CSw_final, p_BW_);


	Eigen::Vector3d rFoot_CSw = rRef_CSw + vRef_CSw*dt + rFootOffset_CSw;
	rFoot_CSw(2) = estimatedGroundHeightCSw_[iLeg] + swingFootHeightTrajectory_.evaluate(std::min(swingPhase + dt, 1.0));

	return rFoot_CSw;
}

Eigen::Vector3d FootPlacementStrategyInvertedPendulum::getCurrentFootPositionFromPredictedFootHoldLocation(double phase, const Eigen::Vector3d& footLocationAtLiftOffCSw, const Eigen::Vector3d& rFootHold_CSw, const RotationQuaternion& p_BW)
{
  const Eigen::Vector3d rFootHold_CSmb = p_BW.rotate(rFootHold_CSw);
  const Eigen::Vector3d rFootLiftOff_CSmb =  p_BW.rotate(footLocationAtLiftOffCSw);
  const Eigen::Vector3d result = p_BW.inverseRotate(Eigen::Vector3d(getSagittalComponentOfFootStep(phase, rFootLiftOff_CSmb.x(), rFootHold_CSmb.x()), getCoronalComponentOfFootStep(phase,  rFootLiftOff_CSmb.y(), rFootHold_CSmb.y()),0.0));
	return 	result;
}


double FootPlacementStrategyInvertedPendulum::getCoronalComponentOfFootStep(double phase, double initialStepOffset, double stepGuess)
{
	//we want the step, for the first part of the motion, to be pretty conservative, and towards the end pretty aggressive in terms of stepping
	//at the desired feedback-based foot position
	phase = mapTo01Range(phase-0.3, 0, 0.5);
	double result = stepGuess * phase + initialStepOffset * (1-phase);
	return result;
}

double FootPlacementStrategyInvertedPendulum::getSagittalComponentOfFootStep(double phase, double initialStepOffset, double stepGuess)
{
	phase = stepInterpolationFunction.evaluate_linear(phase);
	double result = stepGuess * phase + initialStepOffset * (1-phase);
	return result;
}

void FootPlacementStrategyInvertedPendulum::update(robotModel::RobotModel* robotModel)
{

}




} // namespace loco
