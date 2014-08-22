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
//  std::cout << "Leg: " << leg->getId() << std::endl;

  const RotationQuaternion orientationWorldToBaseInWorldFrame = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();
  const RotationQuaternion orientationWorldToHeading = torso_->getMeasuredState().getWorldToHeadingOrientation();
  const RotationQuaternion orientationHeadingToBase = torso_->getMeasuredState().getHeadingToBaseOrientation();


  LinearVelocity desiredLinearVelocityBaseInHeadingFrame = orientationHeadingToBase.inverseRotate(torso_->getDesiredState().getBaseLinearVelocityInBaseFrame());
  desiredLinearVelocityBaseInHeadingFrame.y() = 0.0;
  desiredLinearVelocityBaseInHeadingFrame.z() = 0.0;

  LinearVelocity desiredLinearVelocityBaseInWorldFrame = orientationWorldToHeading.inverseRotate(desiredLinearVelocityBaseInHeadingFrame);

//  const double desiredHeadingSpeedInBaseFrame = orientationHeadingToBase.inverseRotate(torso_->getDesiredState().getBaseLinearVelocityInBaseFrame());
//  std::cout << "speed: " << desiredHeadingSpeedInBaseFrame  << std::endl;

  double swingPhase = 1;
  if (leg->isInSwingMode()) {
    swingPhase = leg->getSwingPhase();
  }


//  const Position rFootHoldOffset_CSw_default = orientationWorldToBaseInWorldFrame.inverseRotate(leg->getProperties().getDesiredDefaultSteppingPositionHipToFootInBaseFrame()); // todo
  const Position desiredDefaultSteppingPositionHipToFootInHeadingFrame = leg->getProperties().getDesiredDefaultSteppingPositionHipToFootInHeadingFrame();
  const Position defaultPositionHipToFootHoldInWorldFrame = orientationWorldToHeading.inverseRotate(desiredDefaultSteppingPositionHipToFootInHeadingFrame); // todo




	/* inverted pendulum stepping offset */
	const Position rRef_CSw = leg->getWorldToHipPositionInWorldFrame();
//	LinearVelocity vRef_CSw = (leg->getHipLinearVelocityInWorldFrame()+orientationWorldToBaseInWorldFrame.inverseRotate(torso_->getMeasuredState().getBaseLinearVelocityInBaseFrame()))/2.0;
	LinearVelocity vRef_CSw = (leg->getHipLinearVelocityInWorldFrame()+orientationWorldToBaseInWorldFrame.inverseRotate(torso_->getMeasuredState().getBaseLinearVelocityInBaseFrame()))/2.0;
	const Position invertedPendulumStepLocation_CSw = rRef_CSw;
//	std::cout << "height: " << getHeightOfTerrainInWorldFrame(invertedPendulumStepLocation_CSw) << std::endl;
	double invertedPendulumHeight = std::max((rRef_CSw.z() - getHeightOfTerrainInWorldFrame(invertedPendulumStepLocation_CSw)), 0.0);
//	LinearVelocity vBaseDes_CSmb(desiredHeadingSpeedInBaseFrame, 0, 0);

//	LinearVelocity vError_CSw = vRef_CSw - orientationWorldToBaseInWorldFrame.inverseRotate(vBaseDes_CSmb); // do not use A_WB*vBaseDes_CSmb
//  LinearVelocity vError_CSw = vRef_CSw - orientationWorldToHeading.inverseRotate(vBaseDes_CSmb); // do not use A_WB*vBaseDes_CSmb

  LinearVelocity vError_CSw = vRef_CSw -desiredLinearVelocityBaseInWorldFrame;

	const double gravity = torso_->getProperties().getGravity().norm();

	Position invertedPendulumPositionHipToFootHoldInWorldFrame = Position(vError_CSw)*std::sqrt(invertedPendulumHeight/gravity);

	/* limit offset from inverted pendulum */
//	const double legLengthMax = 0.5*leg->getProperties().getLegLength();
	invertedPendulumPositionHipToFootHoldInWorldFrame.z() = 0.0;
//	double desLegLength = rFootHoldOffset_CSw_invertedPendulum.norm();
//	if (desLegLength != 0.0) {
//	  double boundedLegLength = desLegLength;
//	  boundToRange(&boundedLegLength, -legLengthMax, legLengthMax);
//	  rFootHoldOffset_CSw_invertedPendulum *= boundedLegLength/desLegLength;
//	}


	/* feedforward stepping offset */
	//we also need to add a desired-velocity dependent feed forward step length
	double netCOMDisplacementPerStride = desiredLinearVelocityBaseInHeadingFrame.x() * leg->getStanceDuration();
	//this is relative to the hip location, hence the divide by 2.0
	double feedForwardStepLength = netCOMDisplacementPerStride / 2.0;

//	Position rFootHoldOffset_CSw_feedforward = orientationWorldToBaseInWorldFrame.inverseRotate(Position(feedForwardStepLength, 0.0, 0.0));
  Position feedForwardPositionHipToFootHoldInWorldFrame = orientationWorldToHeading.inverseRotate(Position(feedForwardStepLength, 0.0, 0.0));

//  std::cout << "footholdoffset CSw: " << rFootHoldOffset_CSw_default<< std::endl;
//	std::cout << "feedforward: " << feedForwardPositionHipToFootHoldInWorldFrame << std::endl;
//  std::cout <<  "pendulum: " << rFootHoldOffset_CSw_invertedPendulum << std::endl;

	Position rFootHoldOffset_CSw_final = defaultPositionHipToFootHoldInWorldFrame + feedForwardPositionHipToFootHoldInWorldFrame + stepFeedbackScale_*invertedPendulumPositionHipToFootHoldInWorldFrame;
//	Position rFootHoldOffset_CSw_final = rFootHoldOffset_CSw_feedforward + stepFeedbackScale_*rFootHoldOffset_CSw_invertedPendulum;
//	std::cout << leg->getId() << ": footOffsetFinalCSw: " << rFootHoldOffset_CSw_final << std::endl;
	rFootHoldOffset_CSw_final.z() = 0.0;
	const Position positionHipToFootInWorldFrameAtLiftOff = leg->getStateLiftOff()->getFootPositionInWorldFrame()-leg->getStateLiftOff()->getHipPositionInWorldFrame();
	Position rFootOffset_CSw = getCurrentFootPositionFromPredictedFootHoldLocationInWorldFrame(std::min(swingPhase + tinyTimeStep, 1.0),  positionHipToFootInWorldFrameAtLiftOff, rFootHoldOffset_CSw_final, leg);
//  std::cout << leg->getId() << ": footOffsetCurrentCSw: " << rFootHoldOffset_CSw_final << std::endl;
//  Position rFoot_CSw = rRef_CSw + rFootHoldOffset_CSw_default + (Position(vRef_CSw)*tinyTimeStep + rFootOffset_CSw);

//  std::cout << "interpolated: " << rFootOffset_CSw << std::endl;

  Position rFoot_CSw = rRef_CSw + (Position(vRef_CSw)*tinyTimeStep + rFootOffset_CSw);
  positionWorldToFootHoldInWorldFrame_[leg->getId()] = rRef_CSw + (Position(vRef_CSw)*tinyTimeStep + rFootHoldOffset_CSw_final);
  positionWorldToFootHoldInWorldFrame_[leg->getId()].z() = getHeightOfTerrainInWorldFrame(positionWorldToFootHoldInWorldFrame_[leg->getId()]);

  positionWorldToFootHoldInvertedPendulumInWorldFrame_[leg->getId()] =  rRef_CSw + Position(vRef_CSw)*tinyTimeStep + defaultPositionHipToFootHoldInWorldFrame;
  positionWorldToFootHoldInvertedPendulumInWorldFrame_[leg->getId()].z() = getHeightOfTerrainInWorldFrame(positionWorldToFootHoldInvertedPendulumInWorldFrame_[leg->getId()]);
//std::cout << "foot pos: " << positionWorldToFootHoldInWorldFrame_[leg->getId()] << std::endl;
  // to avoid slippage, do not move the foot in the horizontal plane when the leg is still grounded
	if (leg->isGrounded()) {
	  rFoot_CSw = leg->getWorldToFootPositionInWorldFrame();
	}

	rFoot_CSw.z() = getHeightOfTerrainInWorldFrame(rFoot_CSw) + swingFootHeightTrajectory_.evaluate(std::min(swingPhase + tinyTimeStep, 1.0));

	return rFoot_CSw;

}

Position FootPlacementStrategyInvertedPendulum::getCurrentFootPositionFromPredictedFootHoldLocationInWorldFrame(double swingPhase, const Position& positionHipToFootAtLiftOffInWorldFrame, const Position& positionHipToFootAtNextTouchDownInWorldFrame, LegBase* leg)
{
  // rotate positions to base frame to interpolate each direction (lateral and heading) individually
//  const RotationQuaternion orientationWorldToBaseInWorldFrame = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();
  const RotationQuaternion orientationWorldToHeading = torso_->getMeasuredState().getWorldToHeadingOrientation();


  Position predictedPositionHipToFootHoldInHeadingFrame = orientationWorldToHeading.rotate(positionHipToFootAtNextTouchDownInWorldFrame);
  const Position positionHipToFootHoldAtLiftOffInHeadingFrame =  orientationWorldToHeading.rotate(positionHipToFootAtLiftOffInWorldFrame);
  return orientationWorldToHeading.inverseRotate(Position(getHeadingComponentOfFootStep(swingPhase, positionHipToFootHoldAtLiftOffInHeadingFrame.x(), predictedPositionHipToFootHoldInHeadingFrame.x(), leg), getLateralComponentOfFootStep(swingPhase,  positionHipToFootHoldAtLiftOffInHeadingFrame.y(), predictedPositionHipToFootHoldInHeadingFrame.y(), leg),0.0));
}


double FootPlacementStrategyInvertedPendulum::getLateralComponentOfFootStep(double phase, double initialStepOffset, double stepGuess, LegBase* leg)
{
	//we want the step, for the first part of the motion, to be pretty conservative, and towards the end pretty aggressive in terms of stepping
	//at the desired feedback-based foot position
	phase = mapTo01Range(phase-0.3, 0, 0.5);
//	return stepGuess * phase + initialStepOffset * (1-phase);
	double result = stepGuess * phase + initialStepOffset * (1.0-phase);
	const double legLength =  leg->getProperties().getLegLength();
  boundToRange(&result, -legLength * 0.5, legLength * 0.5);
  return result;
}

double FootPlacementStrategyInvertedPendulum::getHeadingComponentOfFootStep(double phase, double initialStepOffset, double stepGuess, LegBase* leg)
{
	phase = stepInterpolationFunction_.evaluate_linear(phase);
//	return stepGuess * phase + initialStepOffset * (1-phase);
	double result = stepGuess * phase + initialStepOffset * (1.0-phase);
	const double legLength =  leg->getProperties().getLegLength();
  const double sagittalMaxLegLengthScale = 0.5;
  boundToRange(&result, -legLength * sagittalMaxLegLengthScale, legLength * sagittalMaxLegLengthScale);
  return result;
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
    legs_->getLeftForeLeg()->getProperties().setDesiredDefaultSteppingPositionHipToFootInHeadingFrame(leftOffset);
    legs_->getRightForeLeg()->getProperties().setDesiredDefaultSteppingPositionHipToFootInHeadingFrame(rightOffset);

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
    legs_->getLeftHindLeg()->getProperties().setDesiredDefaultSteppingPositionHipToFootInHeadingFrame(leftOffset);
    legs_->getRightHindLeg()->getProperties().setDesiredDefaultSteppingPositionHipToFootInHeadingFrame(rightOffset);
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
      leg->setDesireWorldToFootPositionInWorldFrame(positionWorldToFootInWorldFrame); // for debugging
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

bool FootPlacementStrategyInvertedPendulum::setToInterpolated(const FootPlacementStrategyBase& footPlacementStrategy1, const FootPlacementStrategyBase& footPlacementStrategy2, double t) {
  const FootPlacementStrategyInvertedPendulum& footPlacement1 = static_cast<const FootPlacementStrategyInvertedPendulum&>(footPlacementStrategy1);
  const FootPlacementStrategyInvertedPendulum& footPlacement2 = static_cast<const FootPlacementStrategyInvertedPendulum&>(footPlacementStrategy2);
  this->stepFeedbackScale_ = linearlyInterpolate(footPlacement1.stepFeedbackScale_, footPlacement2.stepFeedbackScale_, 0.0, 1.0, t);
  if (!interpolateHeightTrajectory(this->swingFootHeightTrajectory_, footPlacement1.swingFootHeightTrajectory_, footPlacement2.swingFootHeightTrajectory_, t)) {
    return false;
  }
  return true;
}

bool FootPlacementStrategyInvertedPendulum::interpolateHeightTrajectory(rbf::BoundedRBF1D& interpolatedTrajectory, const rbf::BoundedRBF1D& trajectory1, const rbf::BoundedRBF1D& trajectory2, double t) {

  const int nKnots = std::max<int>(trajectory1.getKnotCount(), trajectory2.getKnotCount());
  const double tMax1 = trajectory1.getKnotPosition(trajectory1.getKnotCount()-1);
  const double tMax2 = trajectory2.getKnotPosition(trajectory2.getKnotCount()-1);
  const double tMin1 = trajectory1.getKnotPosition(0);
  const double tMin2 = trajectory2.getKnotPosition(0);
  double tMax = std::max<double>(tMax1, tMax2);
  double tMin = std::min<double>(tMin1, tMin2);

  double dt = (tMax-tMin)/(nKnots-1);
  std::vector<double> tValues, xValues;
  for (int i=0; i<nKnots;i++){
    const double time = tMin + i*dt;
    double v = linearlyInterpolate(trajectory1.evaluate(time), trajectory2.evaluate(time), 0.0, 1.0, t);
    tValues.push_back(time);
    xValues.push_back(v);
//    printf("(%f,%f / %f, %f) ", time,v, legProps1->swingFootHeightTrajectory.evaluate(time), legProps2->swingFootHeightTrajectory.evaluate(time));
  }

  interpolatedTrajectory.setRBFData(tValues, xValues);
  return true;
}


} // namespace loco
