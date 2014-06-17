/*!
 * @file   FootPlacementStrategyJump.cpp
 * @author wko
 * @date   Jun 6, 2014
 * @version  1.0
 * @ingroup  robotTask
 * @brief
 */

#include "loco/foot_placement_strategy/FootPlacementStrategyJump.hpp"
#include "loco/common/TorsoBase.hpp"

#include "loco/temp_helpers/math.hpp"

namespace loco {

FootPlacementStrategyJump::FootPlacementStrategyJump(
    LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain)
    : FootPlacementStrategyBase(),
      legs_(legs),
      torso_(torso),
      terrain_(terrain) {

  stepFeedbackScale_ = 1.1;
  stepInterpolationFunction_.clear();
  stepInterpolationFunction_.addKnot(0, 0);
  stepInterpolationFunction_.addKnot(0.6, 1);
}

FootPlacementStrategyJump::~FootPlacementStrategyJump() {

}

bool FootPlacementStrategyJump::loadParameters(const TiXmlHandle& handle) {
  TiXmlElement* pElem;

  /* desired */
  TiXmlHandle hFPS(
      handle.FirstChild("FootPlacementStrategy").FirstChild("Jump"));
  pElem = hFPS.Element();
  if (!pElem) {
    printf("Could not find FootPlacementStrategy:Jump\n");
    return false;
  }

  pElem = hFPS.FirstChild("Gains").Element();
  if (pElem->QueryDoubleAttribute("feedbackScale", &stepFeedbackScale_)
      != TIXML_SUCCESS) {
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
    if (pElem->QueryDoubleAttribute("heading", &offsetHeading)
        != TIXML_SUCCESS) {
      printf("Could not find Offset:Fore:heading!\n");
      return false;
    }

    if (pElem->QueryDoubleAttribute("lateral", &offsetLateral)
        != TIXML_SUCCESS) {
      printf("Could not find Offset:Fore:lateral!\n");
      return false;
    }
    Position leftOffset(offsetHeading, offsetLateral, 0.0);
    Position rightOffset(offsetHeading, -offsetLateral, 0.0);
    legs_->getLeftForeLeg()->getProperties()
        .setDesiredDefaultSteppingPositionHipToFootInHeadingFrame(leftOffset);
    legs_->getRightForeLeg()->getProperties()
        .setDesiredDefaultSteppingPositionHipToFootInHeadingFrame(rightOffset);

  }

  {
    pElem = hFPS.FirstChild("Offset").FirstChild("Hind").Element();
    if (!pElem) {
      printf("Could not find Offset:Hind\n");
      return false;
    }
    double offsetHeading = 0.0;
    double offsetLateral = 0.0;
    if (pElem->QueryDoubleAttribute("heading", &offsetHeading)
        != TIXML_SUCCESS) {
      printf("Could not find Offset:Hind:heading!\n");
      return false;
    }

    if (pElem->QueryDoubleAttribute("lateral", &offsetLateral)
        != TIXML_SUCCESS) {
      printf("Could not find Offset:Hind:lateral!\n");
      return false;
    }
    Position leftOffset(offsetHeading, offsetLateral, 0.0);
    Position rightOffset(offsetHeading, -offsetLateral, 0.0);
    legs_->getLeftHindLeg()->getProperties()
        .setDesiredDefaultSteppingPositionHipToFootInHeadingFrame(leftOffset);
    legs_->getRightHindLeg()->getProperties()
        .setDesiredDefaultSteppingPositionHipToFootInHeadingFrame(rightOffset);
  }

  /* height trajectory */
  if (!loadHeightTrajectory(hFPS.FirstChild("HeightTrajectory"))) {
    return false;
  }

  return true;
}

bool FootPlacementStrategyJump::loadHeightTrajectory(
    const TiXmlHandle &hTrajectory) {
  TiXmlElement* pElem;
  int iKnot;
  double t, value;
  std::vector<double> tValues, xValues;

  TiXmlElement* child = hTrajectory.FirstChild().ToElement();
  for (child; child; child = child->NextSiblingElement()) {
    if (child->QueryDoubleAttribute("t", &t) != TIXML_SUCCESS) {
      printf("Could not find t of knot!\n");
      return false;
    }
    if (child->QueryDoubleAttribute("v", &value) != TIXML_SUCCESS) {
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

bool FootPlacementStrategyJump::initialize(double dt) {
  return true;
}

void FootPlacementStrategyJump::advance(double dt) {
  const Position leftForeLandingPosition(0.3, 0.25, -0.42);
  const Position leftHindLandingPosition(-0.3, 0.25, -0.42);
  const Position rightForeLandingPosition(0.3, -0.25, -0.42);
  const Position rightHindLandingPosition(-0.3, -0.25, -0.42);

  LegBase* leftForeleg = legs_->getLeftForeLeg();
  LegBase* rightForeleg = legs_->getRightForeLeg();
  LegBase* leftHindleg = legs_->getLeftHindLeg();
  LegBase* rightHindleg = legs_->getRightHindLeg();

  if (leftForeleg->isInSwingMode() && !leftForeleg->isGrounded()) {
    leftForeleg->setDesiredJointPositions(
        leftForeleg->getJointPositionsFromBaseToFootPositionInBaseFrame(
            leftForeLandingPosition));
  }
  if (rightForeleg->isInSwingMode() && !rightForeleg->isGrounded()) {
    rightForeleg->setDesiredJointPositions(
        rightForeleg->getJointPositionsFromBaseToFootPositionInBaseFrame(
            rightForeLandingPosition));
  }
  if (leftHindleg->isInSwingMode() && !leftHindleg->isGrounded()) {
    leftHindleg->setDesiredJointPositions(
        leftHindleg->getJointPositionsFromBaseToFootPositionInBaseFrame(
            leftHindLandingPosition));
  }
  if (rightHindleg->isInSwingMode() && !rightHindleg->isGrounded()) {
    rightHindleg->setDesiredJointPositions(
        rightHindleg->getJointPositionsFromBaseToFootPositionInBaseFrame(
            rightHindLandingPosition));
  }
}

double FootPlacementStrategyJump::getHeightOfTerrainInWorldFrame(
    const Position& steppingLocationCSw) {
  Position position = steppingLocationCSw;
  terrain_->getHeight(position);
  return position.z();
}

}  // namespace loco
