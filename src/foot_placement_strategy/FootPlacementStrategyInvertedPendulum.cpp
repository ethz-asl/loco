/*!
* @file 	FootPlacementStrategyInvertedPendulum.cpp
* @author 	Christian Gehring, Stelian Coros
* @date		Sep 7, 2012
* @version 	1.0
* @ingroup 	robotTask
* @brief
*/

#include "loco/foot_placement_strategy/FootPlacementStrategyInvertedPendulum.hpp"
#include "loco/common/TorsoBase.hpp"

#include "loco/temp_helpers/math.hpp"



namespace loco {

FootPlacementStrategyInvertedPendulum::FootPlacementStrategyInvertedPendulum(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain) :
    FootPlacementStrategyBase(),
    legs_(legs),
    torso_(torso),
    terrain_(terrain)
{

	stepFeedbackScale_ = 1.1;
	stepInterpolationFunction_.clear();
	stepInterpolationFunction_.addKnot(0, 0);
	stepInterpolationFunction_.addKnot(0.6, 1);


//	swingFootHeightTrajectory_.clear();
//	swingFootHeightTrajectory_.addKnot(0, 0);
//	swingFootHeightTrajectory_.addKnot(0.65, 0.09);
//	swingFootHeightTrajectory_.addKnot(1.0, 0);



}

FootPlacementStrategyInvertedPendulum::~FootPlacementStrategyInvertedPendulum() {

}

Position FootPlacementStrategyInvertedPendulum::getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep)
{

  const RotationQuaternion orientationWorldToBaseInWorldFrame = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();
  const double desiredHeadingSpeedInBaseFrame = torso_->getDesiredState().getHeadingSpeedInBaseFrame();


  double swingPhase = 1;
  if (leg->isInSwingMode()) {
    swingPhase = leg->getSwingPhase();
  }


  const Position rFootHoldOffset_CSw_default = orientationWorldToBaseInWorldFrame.inverseRotate(leg->getProperties().getDesiredDefaultSteppingPositionHipToFootInBaseFrame()); // todo


	/* inverted pendulum stepping offset */
	const Position rRef_CSw = leg->getWorldToHipPositionInWorldFrame();
	LinearVelocity vRef_CSw = (leg->getHipLinearVelocityInWorldFrame()+torso_->getMeasuredState().getBaseLinearVelocityInBaseFrame())/2.0;
	const Position invertedPendulumStepLocation_CSw = rRef_CSw;
	double invertedPendulumHeight = std::max((rRef_CSw.z() - getHeightOfTerrainInWorldFrame(invertedPendulumStepLocation_CSw)), 0.0);
	LinearVelocity vBaseDes_CSmb(desiredHeadingSpeedInBaseFrame, 0, 0);
	LinearVelocity vError_CSw = vRef_CSw - orientationWorldToBaseInWorldFrame.inverseRotate(vBaseDes_CSmb); // do not use A_WB*vBaseDes_CSmb
	double gravity = torso_->getProperties().getGravity().norm();
	Position rFootHoldOffset_CSw_invertedPendulum = Position(vError_CSw)*std::sqrt(invertedPendulumHeight/gravity);

	/* feedforward stepping offset */
	//we also need to add a desired-velocity dependent feed forward step length
	double netCOMDisplacementPerStride = desiredHeadingSpeedInBaseFrame * leg->getStanceDuration();
	//this is relative to the hip location, hence the divide by 2.0
	double feedForwardStepLength = netCOMDisplacementPerStride / 2.0;

	Position rFootHoldOffset_CSw_feedforward = orientationWorldToBaseInWorldFrame.inverseRotate(Position(feedForwardStepLength, 0.0, 0.0));


//	Position rFootHoldOffset_CSw_final = rFootHoldOffset_CSw_default + rFootHoldOffset_CSw_feedforward + stepFeedbackScale_*rFootHoldOffset_CSw_invertedPendulum;
	Position rFootHoldOffset_CSw_final = rFootHoldOffset_CSw_feedforward + stepFeedbackScale_*rFootHoldOffset_CSw_invertedPendulum;
//	std::cout << leg->getId() << ": footOffsetFinalCSw: " << rFootHoldOffset_CSw_final << std::endl;
	rFootHoldOffset_CSw_final(2) = 0.0;
	const Position footLocationAtLiftOffCSw = leg->getStateLiftOff()->getHipPositionInWorldFrame()-leg->getStateLiftOff()->getFootPositionInWorldFrame();
	Position rFootOffset_CSw = getCurrentFootPositionFromPredictedFootHoldLocationInWorldFrame(std::min(swingPhase + tinyTimeStep, 1.0),  footLocationAtLiftOffCSw, rFootHoldOffset_CSw_final);
//  std::cout << leg->getId() << ": footOffsetCurrentCSw: " << rFootHoldOffset_CSw_final << std::endl;
  Position rFoot_CSw = rRef_CSw + rFootHoldOffset_CSw_default + (Position(vRef_CSw)*tinyTimeStep + rFootOffset_CSw);


  // to avoid slippage, do not move the foot in the horizontal plane when the leg is still grounded
	if (leg->isGrounded()) {
	  rFoot_CSw = leg->getWorldToFootPositionInWorldFrame();
	}

	rFoot_CSw(2) = getHeightOfTerrainInWorldFrame(rFoot_CSw) + swingFootHeightTrajectory_.evaluate(std::min(swingPhase + tinyTimeStep, 1.0));

	return rFoot_CSw;
}

Position FootPlacementStrategyInvertedPendulum::getCurrentFootPositionFromPredictedFootHoldLocationInWorldFrame(double swingPhase, const Position& positionWorldToFootAtLiftOffInWorldFrame, const Position& positionWorldToFootAtNextTouchDownInWorldFrame)
{
  // rotate positions to base frame to interpolate each direction (lateral and heading) individually
  const RotationQuaternion orientationWorldToBaseInWorldFrame = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();
  const Position rFootHold_CSmb = orientationWorldToBaseInWorldFrame.rotate(positionWorldToFootAtNextTouchDownInWorldFrame);
  const Position rFootLiftOff_CSmb =  orientationWorldToBaseInWorldFrame.rotate(positionWorldToFootAtLiftOffInWorldFrame);
  return orientationWorldToBaseInWorldFrame.inverseRotate(Position(getHeadingComponentOfFootStep(swingPhase, rFootLiftOff_CSmb.x(), rFootHold_CSmb.x()), getLateralComponentOfFootStep(swingPhase,  rFootLiftOff_CSmb.y(), rFootHold_CSmb.y()),0.0));
}


double FootPlacementStrategyInvertedPendulum::getLateralComponentOfFootStep(double phase, double initialStepOffset, double stepGuess)
{
	//we want the step, for the first part of the motion, to be pretty conservative, and towards the end pretty aggressive in terms of stepping
	//at the desired feedback-based foot position
	phase = mapTo01Range(phase-0.3, 0, 0.5);
	return stepGuess * phase + initialStepOffset * (1-phase);
}

double FootPlacementStrategyInvertedPendulum::getHeadingComponentOfFootStep(double phase, double initialStepOffset, double stepGuess)
{
	phase = stepInterpolationFunction_.evaluate_linear(phase);
	return stepGuess * phase + initialStepOffset * (1-phase);
}

bool FootPlacementStrategyInvertedPendulum::loadParameters(const TiXmlHandle& handle) {
  TiXmlElement* pElem;

  /* desired */
  TiXmlHandle hFPS(handle.FirstChild("FootPlacementStrategy").FirstChild("InvertedPendulum"));
  pElem = hFPS.Element();
  if (!pElem) {
    printf("Could not find FootPlacementStrategy:InvertedPendulum\n");
    return false;
  }

  pElem = hFPS.FirstChild("Gains").Element();
  if (pElem->QueryDoubleAttribute("feedbackScale", &stepFeedbackScale_)!=TIXML_SUCCESS) {
    printf("Could not find Gains:feedbackScale\n");
    return false;
  }


  /* offset */
  pElem = hFPS.FirstChild("Offset").Element();
  if (!pElem) {
    printf("Could not find Offset\n");
    return false;
  }

  {
    pElem = hFPS.FirstChild("Offset").FirstChild("Fore").Element();
    if (!pElem) {
      printf("Could not find Offset:Fore\n");
      return false;
    }
    double offsetHeading = 0.0;
    double offsetLateral = 0.0;
    if (pElem->QueryDoubleAttribute("heading", &offsetHeading)!=TIXML_SUCCESS) {
      printf("Could not find Offset:Fore:heading!\n");
      return false;
    }

    if (pElem->QueryDoubleAttribute("lateral", &offsetLateral)!=TIXML_SUCCESS) {
      printf("Could not find Offset:Fore:lateral!\n");
      return false;
    }
    Position leftOffset(offsetHeading, offsetLateral, 0.0);
    Position rightOffset(offsetHeading, -offsetLateral, 0.0);
    legs_->getLeftForeLeg()->getProperties().setDesiredDefaultSteppingPositionHipToFootInBaseFrame(leftOffset);
    legs_->getRightForeLeg()->getProperties().setDesiredDefaultSteppingPositionHipToFootInBaseFrame(rightOffset);

  }

  {
    pElem = hFPS.FirstChild("Offset").FirstChild("Hind").Element();
    if (!pElem) {
      printf("Could not find Offset:Hind\n");
      return false;
    }
    double offsetHeading = 0.0;
    double offsetLateral = 0.0;
    if (pElem->QueryDoubleAttribute("heading", &offsetHeading)!=TIXML_SUCCESS) {
      printf("Could not find Offset:Hind:heading!\n");
      return false;
    }

    if (pElem->QueryDoubleAttribute("lateral", &offsetLateral)!=TIXML_SUCCESS) {
      printf("Could not find Offset:Hind:lateral!\n");
      return false;
    }
    Position leftOffset(offsetHeading, offsetLateral, 0.0);
    Position rightOffset(offsetHeading, -offsetLateral, 0.0);
    legs_->getLeftHindLeg()->getProperties().setDesiredDefaultSteppingPositionHipToFootInBaseFrame(leftOffset);
    legs_->getRightHindLeg()->getProperties().setDesiredDefaultSteppingPositionHipToFootInBaseFrame(rightOffset);
  }



  /* height trajectory */
  if(!loadHeightTrajectory(hFPS.FirstChild("HeightTrajectory"))) {
    return false;
  }


  return true;
}

bool FootPlacementStrategyInvertedPendulum::loadHeightTrajectory(const TiXmlHandle &hTrajectory)
{
  TiXmlElement* pElem;
  int iKnot;
  double t, value;
  std::vector<double> tValues, xValues;


  TiXmlElement* child = hTrajectory.FirstChild().ToElement();
   for( child; child; child=child->NextSiblingElement() ){
      if (child->QueryDoubleAttribute("t", &t)!=TIXML_SUCCESS) {
        printf("Could not find t of knot!\n");
        return false;
      }
      if (child->QueryDoubleAttribute("v", &value)!=TIXML_SUCCESS) {
        printf("Could not find v of knot!\n");
        return false;
      }
      tValues.push_back(t);
      xValues.push_back(value);
//      swingFootHeightTrajectory_.addKnot(t, value);
//      printf("t=%f, v=%f\n", t, value);
   }
   swingFootHeightTrajectory_.setRBFData(tValues, xValues);


  return true;
}

bool FootPlacementStrategyInvertedPendulum::initialize(double dt) {
  return true;
}

void FootPlacementStrategyInvertedPendulum::advance(double dt)
{

  for (auto leg : *legs_) {
    if (leg->isInSwingMode()) {
      const Position positionWorldToFootInWorldFrame = getDesiredWorldToFootPositionInWorldFrame(leg, 0.0);
      const Position positionBaseToFootInWorldFrame = positionWorldToFootInWorldFrame - torso_->getMeasuredState().getWorldToBasePositionInWorldFrame();
      const Position positionBaseToFootInBaseFrame  = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame().rotate(positionBaseToFootInWorldFrame);

      leg->setDesiredJointPositions(leg->getJointPositionsFromBaseToFootPositionInBaseFrame(positionBaseToFootInBaseFrame));
    }
//    else {
//      leg->setDesiredJointPositions(leg->getMeasuredJointPositions());
//    }

  }

}

double FootPlacementStrategyInvertedPendulum::getHeightOfTerrainInWorldFrame(const Position& steppingLocationCSw)
{
  Position position = steppingLocationCSw;
  terrain_->getHeight(position);
  return position.z();
}




} // namespace loco
