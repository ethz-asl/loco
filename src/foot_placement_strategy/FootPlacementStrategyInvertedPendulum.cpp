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

Position FootPlacementStrategyInvertedPendulum::getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep) {

  //--- Get rotations
  const RotationQuaternion& orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  const RotationQuaternion& orientationControlToBase = torso_->getMeasuredState().getOrientationControlToBase();
  const RotationQuaternion& orientationWorldToBase = torso_->getMeasuredState().getOrientationWorldToBase();
  //---

  //--- Get desired velocity heading component in control frame
  LinearVelocity desiredLinearVelocityBaseInControlFrame = torso_->getDesiredState().getLinearVelocityBaseInControlFrame();
  desiredLinearVelocityBaseInControlFrame.y() = 0.0;
  desiredLinearVelocityBaseInControlFrame.z() = 0.0;
  //---

  double swingPhase = 1;
  if (!leg->isSupportLeg()) {
    swingPhase = leg->getSwingPhase();
  }

  const Position desiredDefaultSteppingPositionHipToFootInControlFrame = leg->getProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame();
  const Position defaultPositionHipToFootHoldInWorldFrame = orientationWorldToControl.inverseRotate(desiredDefaultSteppingPositionHipToFootInControlFrame); // todo

	/* inverted pendulum stepping offset */
	const Position refPositionWorldToHipInWorldFrame = leg->getWorldToHipPositionInWorldFrame();

	//--- i_v_ref = 0.5*(i_v_h+i_v_com)
	LinearVelocity refLinearVelocityAtHipInWorldFrame = (leg->getHipLinearVelocityInWorldFrame()
	                                                     + orientationWorldToBase.inverseRotate(torso_->getMeasuredState().getLinearVelocityBaseInBaseFrame())
	                                                     )/2.0;
	//---

	const Position invertedPendulumStepLocationInWorldFrame = refPositionWorldToHipInWorldFrame;

	const Position& positionWorldToControlInWorldFrame = torso_->getMeasuredState().getPositionWorldToControlInWorldFrame();

	Position positionWorldToHipProjectedToTerrainInWorldFrame = invertedPendulumStepLocationInWorldFrame;

	if (!terrain_->getHeight(positionWorldToHipProjectedToTerrainInWorldFrame)) {
	  throw std::runtime_error("fp could not get terrain");
	}

	Vector surfaceNormalInWorldFrame; // parallel to z-axis of control frame
	terrain_->getNormal(positionWorldToHipProjectedToTerrainInWorldFrame, surfaceNormalInWorldFrame);

	/**********************************
	 * Inverted pendulum contribution *
	 **********************************/
	double invertedPendulumHeightInControlFrame = std::max((refPositionWorldToHipInWorldFrame-positionWorldToHipProjectedToTerrainInWorldFrame).dot(surfaceNormalInWorldFrame), 0.0);

	Position terrainToHipInWorldFrame = invertedPendulumHeightInControlFrame*(loco::Position)surfaceNormalInWorldFrame;

  LinearVelocity linearVelocityErrorInWorldFrame = refLinearVelocityAtHipInWorldFrame - orientationWorldToControl.inverseRotate(desiredLinearVelocityBaseInControlFrame);

//  std::cout << "ref lin vel: " << refLinearVelocityAtHipInWorldFrame
//      << "des lin vel: " << orientationWorldToControl.inverseRotate(desiredLinearVelocityBaseInControlFrame)
//      << std::endl;

	const double gravitationalAccleration = torso_->getProperties().getGravity().norm();

	Position invertedPendulumPositionHipToFootHoldInWorldFrame = orientationWorldToControl.inverseRotate(
	                                                              stepFeedbackScale_
	                                                             * orientationWorldToControl.rotate(Position(linearVelocityErrorInWorldFrame))
	                                                             * std::sqrt(invertedPendulumHeightInControlFrame/gravitationalAccleration)
	                                                            );
//	invertedPendulumPositionHipToFootHoldInWorldFrame = orientationWorldToControl.rotate(invertedPendulumPositionHipToFootHoldInWorldFrame);
//	invertedPendulumPositionHipToFootHoldInWorldFrame.z() = 0.0; // we only want x-y components, the height will be generated by the interpolator
//	invertedPendulumPositionHipToFootHoldInWorldFrame = orientationWorldToControl.inverseRotate(invertedPendulumPositionHipToFootHoldInWorldFrame);

	/* limit offset from inverted pendulum */
//	const double legLengthMax = 0.5*leg->getProperties().getLegLength();

//	double desLegLength = rFootHoldOffset_CSw_invertedPendulum.norm();
//	if (desLegLength != 0.0) {
//	  double boundedLegLength = desLegLength;
//	  boundToRange(&boundedLegLength, -legLengthMax, legLengthMax);
//	  rFootHoldOffset_CSw_invertedPendulum *= boundedLegLength/desLegLength;
//	}

	invertedPendulumPositionHipToFootHoldInWorldFrame_[leg->getId()] = invertedPendulumPositionHipToFootHoldInWorldFrame;
	/**********************************
   * Inverted pendulum contribution *
   **********************************/

  /*******************************
   * feedforward stepping offset *
   *******************************/
  double netCOMDisplacementPerStride = desiredLinearVelocityBaseInControlFrame.x() * leg->getStanceDuration();
  double feedForwardStepLength = netCOMDisplacementPerStride / 2.0;
  Position feedForwardPositionHipToFootHoldInWorldFrame = orientationWorldToControl.inverseRotate(Position(feedForwardStepLength, 0.0, 0.0));
  testingFFhipToFootInWorldFrame_[leg->getId()] = feedForwardPositionHipToFootHoldInWorldFrame; // only for visual debug
  /***********************************
   * end feedforward stepping offset *
   ***********************************/

  testingFBinvertedPendulumContribution_[leg->getId()] = invertedPendulumPositionHipToFootHoldInWorldFrame;

  //--- Find zero height in control frame
	Position positionHipToDesiredFootholdInWorldFrame = defaultPositionHipToFootHoldInWorldFrame + feedForwardPositionHipToFootHoldInWorldFrame + invertedPendulumPositionHipToFootHoldInWorldFrame;
	positionHipToDesiredFootholdInWorldFrame = orientationWorldToControl.rotate(positionHipToDesiredFootholdInWorldFrame);
	positionHipToDesiredFootholdInWorldFrame.z() = 0.0;
	positionHipToDesiredFootholdInWorldFrame = orientationWorldToControl.inverseRotate(positionHipToDesiredFootholdInWorldFrame);
	//---

	//--- Offset contribution from terrain slope
	loco::Position positionWorldToHipOnTerrain = leg->getWorldToHipPositionInWorldFrame();
	terrain_->getHeight(positionWorldToHipOnTerrain);
	double distanceHipToTerrain = (leg->getWorldToHipPositionInWorldFrame() - positionWorldToHipOnTerrain).dot(surfaceNormalInWorldFrame);
	positionHipToDesiredFootholdInWorldFrame -= (loco::Position)surfaceNormalInWorldFrame*distanceHipToTerrain;
	//---

	testingHipToDesiredFootHold_[leg->getId()] = positionHipToDesiredFootholdInWorldFrame;

	const Position positionHipToFootInWorldFrameAtLiftOff = leg->getStateLiftOff()->getFootPositionInWorldFrame()-leg->getStateLiftOff()->getHipPositionInWorldFrame();
	Position positionOffsetFromHipProjectedOnTerrainToDesiredFoot = getCurrentFootPositionFromPredictedFootHoldLocationInWorldFrame(std::min(swingPhase + tinyTimeStep, 1.0),  positionHipToFootInWorldFrameAtLiftOff, positionHipToDesiredFootholdInWorldFrame, leg);

	Position positionWorldToDesiredFootInWorldFrame = leg->getWorldToHipPositionInWorldFrame()
	                                                  - (loco::Position)surfaceNormalInWorldFrame*distanceHipToTerrain
	                                                  + positionOffsetFromHipProjectedOnTerrainToDesiredFoot;


	//Position positionWorldToDesiredFootInWorldFrame = refPositionWorldToHipInWorldFrame + (Position(refLinearVelocityAtHipInWorldFrame)*tinyTimeStep + positionOffsetFromHipProjectedOnTerrainToDesiredFoot);


  //--- logging
  // log the predicted foot hold location which has the the same height as the terrain.
  positionWorldToFootHoldInWorldFrame_[leg->getId()] = refPositionWorldToHipInWorldFrame + (Position(refLinearVelocityAtHipInWorldFrame)*tinyTimeStep + positionHipToDesiredFootholdInWorldFrame);
  terrain_->getHeight(positionWorldToFootHoldInWorldFrame_[leg->getId()],positionWorldToFootHoldInWorldFrame_[leg->getId()].z());

  positionWorldToFootHoldInvertedPendulumInWorldFrame_[leg->getId()] = refPositionWorldToHipInWorldFrame + invertedPendulumPositionHipToFootHoldInWorldFrame;
  terrain_->getHeight(positionWorldToFootHoldInvertedPendulumInWorldFrame_[leg->getId()], positionWorldToFootHoldInvertedPendulumInWorldFrame_[leg->getId()].z());

  positionWorldToDefaultFootHoldInWorldFrame_[leg->getId()] = refPositionWorldToHipInWorldFrame + defaultPositionHipToFootHoldInWorldFrame;
  terrain_->getHeight(positionWorldToDefaultFootHoldInWorldFrame_[leg->getId()]);
  //---

  // to avoid slippage, do not move the foot in the horizontal plane when the leg is still grounded
	if (leg->isGrounded()) {
	  positionWorldToDesiredFootInWorldFrame = leg->getWorldToFootPositionInWorldFrame();
	}

	//positionWorldToDesiredFootInWorldFrame.z() = 0.0;
	//terrain_->getHeight(positionWorldToDesiredFootInWorldFrame);
	positionWorldToDesiredFootInWorldFrame += (loco::Position)surfaceNormalInWorldFrame*swingFootHeightTrajectory_.evaluate(std::min(swingPhase + tinyTimeStep, 1.0));

	//--- Add offset to height to take into accoutn the difference between real foot position and its projection on estimated plane
//	double footPositionAtLiftOffOnFreePlane;
//	terrain_->getHeight(leg->getStateLiftOff()->getFootPositionInWorldFrame(), footPositionAtLiftOffOnFreePlane);
//	double realFootHeightInWorldFrameOffset = leg->getStateLiftOff()->getFootPositionInWorldFrame().z()- footPositionAtLiftOffOnFreePlane;
//	positionWorldToDesiredFootInWorldFrame.z() = realFootHeightInWorldFrameOffset + getHeightOfTerrainInWorldFrame(positionWorldToDesiredFootInWorldFrame) + swingFootHeightTrajectory_.evaluate(std::min(swingPhase + tinyTimeStep, 1.0));
	//---

	heightByTrajectory_[leg->getId()] =  swingFootHeightTrajectory_.evaluate(std::min(swingPhase + tinyTimeStep, 1.0));
	return positionWorldToDesiredFootInWorldFrame;

}

Position FootPlacementStrategyInvertedPendulum::getCurrentFootPositionFromPredictedFootHoldLocationInWorldFrame(double swingPhase, const Position& positionHipToFootAtLiftOffInWorldFrame, const Position& positionHipToFootAtNextTouchDownInWorldFrame, LegBase* leg)
{
  // rotate positions to base frame to interpolate each direction (lateral and heading) individually
//  const RotationQuaternion orientationWorldToBaseInWorldFrame = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();
  const RotationQuaternion orientationWorldToHeading = torso_->getMeasuredState().getOrientationWorldToControl();

  Position predictedPositionHipToFootHoldInHeadingFrame = orientationWorldToHeading.rotate(positionHipToFootAtNextTouchDownInWorldFrame);
  const Position positionHipToFootHoldAtLiftOffInHeadingFrame = orientationWorldToHeading.rotate(positionHipToFootAtLiftOffInWorldFrame);
  return orientationWorldToHeading.inverseRotate(Position(getHeadingComponentOfFootStep(swingPhase,
                                                                                        positionHipToFootHoldAtLiftOffInHeadingFrame.x(),
                                                                                        predictedPositionHipToFootHoldInHeadingFrame.x(),
                                                                                        leg),
                                                          getLateralComponentOfFootStep(swingPhase,
                                                                                        positionHipToFootHoldAtLiftOffInHeadingFrame.y(),
                                                                                        predictedPositionHipToFootHoldInHeadingFrame.y(),
                                                                                        leg),
                                                                                        0.0));
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
      swingFootHeightTrajectory_.addKnot(t, value);
//      printf("t=%f, v=%f\n", t, value);
   }
//   swingFootHeightTrajectory_.setRBFData(tValues, xValues);


  return true;
}

bool FootPlacementStrategyInvertedPendulum::initialize(double dt) {
  return true;
}

void FootPlacementStrategyInvertedPendulum::advance(double dt)
{

  for (auto leg : *legs_) {
	  /* this default desired swing behaviour
	   *
	   */
	  if (!leg->isSupportLeg()) {
		  if (leg->shouldBeGrounded()) {
				  // stance mode according to plan
			  if (leg->isGrounded()) {
				  if (leg->isSlipping()) {
					// not safe to use this leg as support leg
					 regainContact(leg, dt);
						  // todo think harder about this
				  }
					  // torque control

			  }
			  else {
				// not yet touch-down
				// lost contact
				regainContact(leg, dt);
			  }
		  }
		  else {
		  // swing mode according to plan
			  if (leg->isGrounded()) {
				  if (leg->getSwingPhase() <= 0.3) {
					  // leg should lift-off (late lift-off)
						setFootTrajectory(leg);
					  }
				  }
				  else {
					  // leg is on track
					  setFootTrajectory(leg);
				  }
		  }
	  }

  }

}

void FootPlacementStrategyInvertedPendulum::regainContact(LegBase* leg, double dt) {
	Position positionWorldToFootInWorldFrame =  leg->getWorldToFootPositionInWorldFrame();
	double loweringSpeed = 0.05;

	        	loco::Vector normalInWorldFrame;
	        	if (terrain_->getNormal(positionWorldToFootInWorldFrame,normalInWorldFrame)) {
	        		positionWorldToFootInWorldFrame -= 0.01*(loco::Position)normalInWorldFrame;
	        		//positionWorldToFootInWorldFrame -= (loweringSpeed*dt) * (loco::Position)normalInWorldFrame;
	        	}
	        	else  {
	        		throw std::runtime_error("FootPlacementStrategyInvertedPendulum::advance cannot get terrain normal.");
	        	}

	        leg->setDesireWorldToFootPositionInWorldFrame(positionWorldToFootInWorldFrame); // for debugging
	        const Position positionWorldToBaseInWorldFrame = torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
	        const Position positionBaseToFootInWorldFrame = positionWorldToFootInWorldFrame - positionWorldToBaseInWorldFrame;
	        const Position positionBaseToFootInBaseFrame = torso_->getMeasuredState().getOrientationWorldToBase().rotate(positionBaseToFootInWorldFrame);
	        leg->setDesiredJointPositions(leg->getJointPositionsFromBaseToFootPositionInBaseFrame(positionBaseToFootInBaseFrame));

}

void FootPlacementStrategyInvertedPendulum::setFootTrajectory(LegBase* leg) {
    const Position positionWorldToFootInWorldFrame = getDesiredWorldToFootPositionInWorldFrame(leg, 0.0);
    leg->setDesireWorldToFootPositionInWorldFrame(positionWorldToFootInWorldFrame); // for debugging
    const Position positionBaseToFootInWorldFrame = positionWorldToFootInWorldFrame - torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
    const Position positionBaseToFootInBaseFrame  = torso_->getMeasuredState().getOrientationWorldToBase().rotate(positionBaseToFootInWorldFrame);

    leg->setDesiredJointPositions(leg->getJointPositionsFromBaseToFootPositionInBaseFrame(positionBaseToFootInBaseFrame));
}


const LegGroup& FootPlacementStrategyInvertedPendulum::getLegs() const {
  return *legs_;
}

bool FootPlacementStrategyInvertedPendulum::setToInterpolated(const FootPlacementStrategyBase& footPlacementStrategy1, const FootPlacementStrategyBase& footPlacementStrategy2, double t) {
  const FootPlacementStrategyInvertedPendulum& footPlacement1 = static_cast<const FootPlacementStrategyInvertedPendulum&>(footPlacementStrategy1);
  const FootPlacementStrategyInvertedPendulum& footPlacement2 = static_cast<const FootPlacementStrategyInvertedPendulum&>(footPlacementStrategy2);
  this->stepFeedbackScale_ = linearlyInterpolate(footPlacement1.stepFeedbackScale_, footPlacement2.stepFeedbackScale_, 0.0, 1.0, t);
//  if (!interpolateHeightTrajectory(this->swingFootHeightTrajectory_, footPlacement1.swingFootHeightTrajectory_, footPlacement2.swingFootHeightTrajectory_, t)) {
//    return false;
//  }


  int iLeg = 0;
  for (auto leg : *legs_) {
    leg->getProperties().setDesiredDefaultSteppingPositionHipToFootInHeadingFrame(linearlyInterpolate(
      footPlacement1.getLegs().getLeg(iLeg)->getProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame(),
      footPlacement2.getLegs().getLeg(iLeg)->getProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame(),
      0.0,
      1.0,
      t));
    iLeg++;
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

const Position& FootPlacementStrategyInvertedPendulum::getPositionWorldToDesiredFootHoldInWorldFrame(LegBase* leg) const {
  return positionWorldToFootHoldInWorldFrame_[leg->getId()];
}

} // namespace loco
