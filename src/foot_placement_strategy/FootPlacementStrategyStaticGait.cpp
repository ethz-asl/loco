/*
 * FootPlacementStrategyStaticGait.cpp
 *
 *  Created on: Oct 6, 2014
 *      Author: dario
 */


#include "loco/foot_placement_strategy/FootPlacementStrategyStaticGait.hpp"

namespace loco {


FootPlacementStrategyStaticGait::FootPlacementStrategyStaticGait(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain) :
    FootPlacementStrategyFreePlane(legs, torso, terrain)
{

  stepInterpolationFunction_.clear();
  stepInterpolationFunction_.addKnot(0, 0);
  stepInterpolationFunction_.addKnot(1.0, 1);

  stepFeedbackScale_ = 0.0;

}


FootPlacementStrategyStaticGait::~FootPlacementStrategyStaticGait() {

}


Position FootPlacementStrategyStaticGait::getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep) {

  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();

  // Find starting point: hip projected vertically on ground
  Position positionWorldToHipOnPlaneAlongNormalInWorldFrame = leg->getStateLiftOff()->getPositionWorldToHipInWorldFrame();
  terrain_->getHeight(positionWorldToHipOnPlaneAlongNormalInWorldFrame);

  //--- Evaluate the interpolation contributions
  Position positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame = getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(*leg); //x-y
  Position positionDesiredFootOnTerrainToDesiredFootInControlFrame = getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(*leg,
                                                                                                                       positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame); // z
  //---

  //--- Build the world to desired foot position and return it
  Position positionWorldToDesiredFootInWorldFrame = positionWorldToHipOnPlaneAlongNormalInWorldFrame // starting point, hip projected on the plane along world z axis
                                                    + orientationWorldToControl.inverseRotate(positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame) // x-y
                                                    + orientationWorldToControl.inverseRotate(positionDesiredFootOnTerrainToDesiredFootInControlFrame); // z
  //---

  //--- Save for debugging
  positionWorldToHipOnPlaneAlongNormalInWorldFrame_[leg->getId()] = positionWorldToHipOnPlaneAlongNormalInWorldFrame;
  positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame_[leg->getId()] = orientationWorldToControl.inverseRotate(positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame);
  positionDesiredFootOnTerrainToDesiredFootInWorldFrame_[leg->getId()] = orientationWorldToControl.inverseRotate(positionDesiredFootOnTerrainToDesiredFootInControlFrame);
  //---

  return positionWorldToDesiredFootInWorldFrame;
  //---

}

// Evaluate feed forward component
Position FootPlacementStrategyStaticGait::getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame(const LegBase& leg)   {
  double stanceDuration = leg.getStanceDuration();

  Position headingAxisOfControlFrame = Position::UnitX();
  Position positionDesiredFootOnTerrainVelocityOffsetInControlFrame = Position(torso_->getDesiredState().getLinearVelocityBaseInControlFrame().toImplementation().cwiseProduct(headingAxisOfControlFrame.toImplementation()))
                                                                      *stanceDuration*0.5;

  Position positionDesiredFootOnTerrainFeedForwardInControlFrame = positionDesiredFootOnTerrainVelocityOffsetInControlFrame;
  return positionDesiredFootOnTerrainFeedForwardInControlFrame;
}


Position FootPlacementStrategyStaticGait::getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(const LegBase& leg) {
  const RotationQuaternion& orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();

  positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg.getId()] = getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame(leg);
  positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg.getId()] = getPositionDesiredFootHoldOnTerrainFeedBackInControlFrame(leg);

  Position positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame = positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg.getId()]
                                                                                     /*+ positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg.getId()]*/;

  //--- save for debug
  //Position positionWorldToHipOnPlaneAlongNormalInWorldFrame = getPositionProjectedOnPlaneAlongSurfaceNormal(leg.getWorldToHipPositionInWorldFrame());

  Position positionWorldToHipVerticalOnPlaneInWorldFrame = leg.getStateLiftOff().getPositionWorldToHipInWorldFrame();
//  Position positionWorldToHipVerticalOnPlaneInWorldFrame = leg.getWorldToHipPositionInWorldFrame();
  terrain_->getHeight(positionWorldToHipVerticalOnPlaneInWorldFrame);

  positionWorldToFootHoldInWorldFrame_[leg.getId()] = positionWorldToHipVerticalOnPlaneInWorldFrame
                                                      + orientationWorldToControl.inverseRotate(positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame);
  //---

  //--- starting point for trajectory interpolation
  const Position positionHipOnTerrainAlongNormalToFootAtLiftOffInWorldFrame = leg.getStateLiftOff().getPositionWorldToFootInWorldFrame()
                                                                              -leg.getStateLiftOff().getPositionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame();
  const Position positionHipOnTerrainAlongNormalToFootAtLiftOffInControlFrame = orientationWorldToControl.rotate(positionHipOnTerrainAlongNormalToFootAtLiftOffInWorldFrame);
  //---

  double interpolationParameter = getInterpolationPhase(leg);

  Position positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame = Position(
                                                                                  // x
                                                                                  getHeadingComponentOfFootStep(interpolationParameter,
                                                                                  positionHipOnTerrainAlongNormalToFootAtLiftOffInControlFrame.x(),
                                                                                  positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame.x(),
                                                                                  const_cast<LegBase*>(&leg)),
                                                                                  // y
                                                                                  getLateralComponentOfFootStep(interpolationParameter,
                                                                                  positionHipOnTerrainAlongNormalToFootAtLiftOffInControlFrame.y(),
                                                                                  positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame.y(),
                                                                                  const_cast<LegBase*>(&leg)),
                                                                                  // z
                                                                                  0.0
                                                                                  );

  return positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame;
}



} /* namespace loco */

