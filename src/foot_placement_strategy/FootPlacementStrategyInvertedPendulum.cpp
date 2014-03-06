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

FootPlacementStrategyInvertedPendulum::FootPlacementStrategyInvertedPendulum(LegGroup* legs, TorsoBase* torso, robotTerrain::TerrainBase* terrain) :
    FootPlacementStrategyBase(),
    legs_(legs),
    torso_(torso),
    terrain_(terrain)
{

	stepFeedbackScale_ = 1.2;
	stepInterpolationFunction.clear();
	stepInterpolationFunction.addKnot(0, 0);
	stepInterpolationFunction.addKnot(0.6, 1);

	gravity_ = 9.81;

//	swingFootHeightTrajectory_.clear();
//	swingFootHeightTrajectory_.addKnot(0, 0);
//	swingFootHeightTrajectory_.addKnot(0.65, 0.09);
//	swingFootHeightTrajectory_.addKnot(1.0, 0);


  for (int iLeg=0; iLeg < 4; iLeg++) {
    estimatedGroundHeightCSw_[iLeg] = 0.0;
    stanceDuration_[iLeg] = 0.8;
    swingPhase_[iLeg] = 0.0;
    footLocationAtLiftOffCSw_[iLeg]  = Eigen::Vector3d::Zero();
    rHip_CSw_[iLeg] = Eigen::Vector3d::Zero();
    vHip_CSw_[iLeg] = Eigen::Vector3d::Zero();
    steppingOffsetToHip_CSmb_[iLeg]  = Eigen::Vector3d::Zero();
  }

  vBase_CSw_ =  Eigen::Vector3d::Zero();

  desiredHeadingSpeedInBaseFrame_ = 0.0;

}
FootPlacementStrategyInvertedPendulum::FootPlacementStrategyInvertedPendulum() :
    FootPlacementStrategyBase(),
    terrain_(nullptr)
{
  stepFeedbackScale_ = 1.1;
  stepInterpolationFunction.clear();
  stepInterpolationFunction.addKnot(0, 0);
  stepInterpolationFunction.addKnot(0.6, 1);

  gravity_ = 9.81;

//  swingFootHeightTrajectory_.clear();
//  swingFootHeightTrajectory_.addKnot(0, 0);
//  swingFootHeightTrajectory_.addKnot(0.65, 0.09);
//  swingFootHeightTrajectory_.addKnot(1.0, 0);


  for (int iLeg=0; iLeg < 4; iLeg++) {
    estimatedGroundHeightCSw_[iLeg] = 0.0;
    stanceDuration_[iLeg] = 0.8;
    swingPhase_[iLeg] = 0.0;
    footLocationAtLiftOffCSw_[iLeg]  = Eigen::Vector3d::Zero();
    rHip_CSw_[iLeg] = Eigen::Vector3d::Zero();
    vHip_CSw_[iLeg] = Eigen::Vector3d::Zero();
    steppingOffsetToHip_CSmb_[iLeg]  = Eigen::Vector3d::Zero();
  }

  vBase_CSw_ =  Eigen::Vector3d::Zero();

  desiredHeadingSpeedInBaseFrame_ = 0.0;
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

void FootPlacementStrategyInvertedPendulum::setSwingFootHeightTrajectory(const SwingFootHeightTrajectory& swingFootHeightTrajectory)
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

void FootPlacementStrategyInvertedPendulum::setSteppingOffsetToHip(int iLeg, const Eigen::Vector3d& steppingOffsetToHip_CSh) {
  steppingOffsetToHip_CSmb_[iLeg] = steppingOffsetToHip_CSh;
}

void FootPlacementStrategyInvertedPendulum::setDesiredHeadingSpeed(double desiredForwardSpeed) {
  desiredHeadingSpeedInBaseFrame_ = desiredForwardSpeed;
}

void FootPlacementStrategyInvertedPendulum::setFeedbackScale(double scale) {
  stepFeedbackScale_ = scale;
}




Position FootPlacementStrategyInvertedPendulum::getDesiredWorldToFootPositionInWorldFrame(int iLeg, double dt)
{

	const double swingPhase = swingPhase_[iLeg];
  Eigen::Vector3d rFootHoldOffset_CSw_default = p_BW_.inverseRotate(steppingOffsetToHip_CSmb_[iLeg]);
//  Eigen::Vector3d rFootHoldOffset_CSw_default = steppingOffsetToHip_CSmb_[iLeg];
//  Eigen::Vector3d rFootHoldOffset_CSw_default = Eigen::Vector3d::Zero();

	/* inverted pendulum stepping offset */
	Eigen::Vector3d rRef_CSw = rHip_CSw_[iLeg];
	Eigen::Vector3d vRef_CSw = (vHip_CSw_[iLeg]+vBase_CSw_)/2.0;
	const Eigen::Vector3d invertedPendulumStepLocation_CSw = rRef_CSw;
	double invertedPendulumHeight = std::max((rRef_CSw.z() - getFootHeightOverTerrain(iLeg, invertedPendulumStepLocation_CSw)), 0.0);
	Eigen::Vector3d vBaseDes_CSmb = Eigen::Vector3d(desiredHeadingSpeedInBaseFrame_, 0, 0);
	Eigen::Vector3d vError_CSw = vRef_CSw - p_BW_.inverseRotate(vBaseDes_CSmb); // do not use A_WB*vBaseDes_CSmb
	Eigen::Vector3d rFootHoldOffset_CSw_invertedPendulum = vError_CSw*sqrt(invertedPendulumHeight/gravity_);

	/* feedforward stepping offset */
	//we also need to add a desired-velocity dependent feed forward step length
	double netCOMDisplacementPerStride = desiredHeadingSpeedInBaseFrame_ * stanceDuration_[iLeg];
	//this is relative to the hip location, hence the divide by 2.0
	double feedForwardStepLength = netCOMDisplacementPerStride / 2.0;

	Eigen::Vector3d rFootHoldOffset_CSw_feedforward = p_BW_.inverseRotate(Eigen::Vector3d(feedForwardStepLength, 0.0, 0.0));


	Eigen::Vector3d rFootHoldOffset_CSw_final = rFootHoldOffset_CSw_default + rFootHoldOffset_CSw_feedforward + stepFeedbackScale_*rFootHoldOffset_CSw_invertedPendulum;
	rFootHoldOffset_CSw_final(2) = 0.0;
	Eigen::Vector3d rFootOffset_CSw = getCurrentFootPositionFromPredictedFootHoldLocation(std::min(swingPhase + dt, 1.0),  footLocationAtLiftOffCSw_[iLeg], rFootHoldOffset_CSw_final, p_BW_);


	Eigen::Vector3d rFoot_CSw = rRef_CSw + vRef_CSw*dt + rFootOffset_CSw;
	rFoot_CSw(2) = getFootHeightOverTerrain(iLeg, rFoot_CSw) + swingFootHeightTrajectory_.evaluate(std::min(swingPhase + dt, 1.0));

	return Position(rFoot_CSw);
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
    Eigen::Vector3d leftOffset(offsetHeading, offsetLateral, 0.0);
    Eigen::Vector3d rightOffset(offsetHeading, -offsetLateral, 0.0);
    this->setSteppingOffsetToHip(0, leftOffset);
    this->setSteppingOffsetToHip(1, rightOffset);
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
    Eigen::Vector3d leftOffset(offsetHeading, offsetLateral, 0.0);
    Eigen::Vector3d rightOffset(offsetHeading, -offsetLateral, 0.0);
    this->setSteppingOffsetToHip(2, leftOffset);
    this->setSteppingOffsetToHip(3, rightOffset);
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
      printf("t=%f, v=%f\n", t, value);
   }
//   swingFootHeightTrajectory_.setRBFData(tValues, xValues);


  return true;
}

bool FootPlacementStrategyInvertedPendulum::initialize(double dt) {
  return true;
}

void FootPlacementStrategyInvertedPendulum::advance(double dt)
{
  int iLeg=0;
  for (auto leg : *legs_) {
    this->setStanceDuration(iLeg, leg->getStanceDuration());
    double swingPhase = 1;
    if (leg->isInSwingMode()) {
      swingPhase = leg->getSwingPhase();
    }
    this->setSwingPhase(iLeg, swingPhase);
    this->setHipPosition(iLeg, leg->getWorldToHipPositionInWorldFrame().toImplementation());
    this->setHipVelocity(iLeg, leg->getHipLinearVelocityInWorldFrame().toImplementation());

    this->setFootLocationAtLiftOff(iLeg, (leg->getStateLiftOff()->getHipPositionInWorldFrame()-leg->getStateLiftOff()->getFootPositionInWorldFrame()).toImplementation());





    // todo
    //  footPlacementTest.setSteppingOffsetToHip(iLeg, Eigen::Vector3d(leg->legProps->steppingOffset.x, leg->legProps->steppingOffset.y, leg->legProps->steppingOffset.z));
    iLeg++;
  }

  this->setDesiredHeadingSpeed(torso_->getDesiredState().getHeadingSpeedInBaseFrame());
  this->setBaseVelocity(iLeg, torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame().inverseRotate(torso_->getMeasuredState().getBaseLinearVelocityInBaseFrame().toImplementation()));
  this->setRotationWorldToBase(torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame());


  iLeg = 0;
  for (auto leg : *legs_) {
    const Position positionWorldToFootInWorldFrame = getDesiredWorldToFootPositionInWorldFrame(iLeg, 0.0);
    const Position positionBaseToFootInWorldFrame = positionWorldToFootInWorldFrame - torso_->getMeasuredState().getWorldToBasePositionInWorldFrame();
    const Position positionBaseToFootInBaseFrame  = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame().rotate(positionBaseToFootInWorldFrame);
    leg->setDesiredJointPositions(leg->getJointPositionsFromBaseToFootPositionInBaseFrame(positionBaseToFootInBaseFrame));
    iLeg++;
  }


}

double FootPlacementStrategyInvertedPendulum::getFootHeightOverTerrain(int iLeg, const Eigen::Vector3d& steppingLocationCSw)
{
  if (terrain_ == nullptr) {
    return estimatedGroundHeightCSw_[iLeg];
  } else  {
    Eigen::Vector3d position = steppingLocationCSw;
    terrain_->getHeight(position);
    estimatedGroundHeightCSw_[iLeg] = position.z();
    return position.z();
  }
}




} // namespace loco
