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

//  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
//
//  // Find starting point: hip projected vertically on ground
//  Position positionWorldToHipOnPlaneAlongNormalInWorldFrame = leg->getStateLiftOff()->getPositionWorldToHipInWorldFrame();
//  terrain_->getHeight(positionWorldToHipOnPlaneAlongNormalInWorldFrame);
//
//  //--- Evaluate the interpolation contributions
//  Position positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame = getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(*leg); //x-y
//  Position positionDesiredFootOnTerrainToDesiredFootInControlFrame = getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(*leg,
//                                                                                                                       positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame); // z
//  //---
//
//  //--- Build the world to desired foot position and return it
//  Position positionWorldToDesiredFootInWorldFrame = positionWorldToHipOnPlaneAlongNormalInWorldFrame // starting point, hip projected on the plane along world z axis
//                                                    + orientationWorldToControl.inverseRotate(positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame) // x-y
//                                                    + orientationWorldToControl.inverseRotate(positionDesiredFootOnTerrainToDesiredFootInControlFrame); // z
//
////  std::cout << "foot components for leg: " << leg->getId() << std::endl
////            << "world to hip: " << positionWorldToHipOnPlaneAlongNormalInWorldFrame << std::endl
////            << "hip on t to des foot: " << orientationWorldToControl.inverseRotate(positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame) << std::endl
////            << "foot to height: " << orientationWorldToControl.inverseRotate(positionDesiredFootOnTerrainToDesiredFootInControlFrame) << std::endl;
//
//  //---
//
//  //--- Save for debugging
//  positionWorldToHipOnPlaneAlongNormalInWorldFrame_[leg->getId()] = positionWorldToHipOnPlaneAlongNormalInWorldFrame;
//  positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame_[leg->getId()] = orientationWorldToControl.inverseRotate(positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame);
//  positionDesiredFootOnTerrainToDesiredFootInWorldFrame_[leg->getId()] = orientationWorldToControl.inverseRotate(positionDesiredFootOnTerrainToDesiredFootInControlFrame);
//  //---

  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  Position positionWorldToFootOnTerrainAtLiftOffInWorldFrame = leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame();
  terrain_->getHeight(positionWorldToFootOnTerrainAtLiftOffInWorldFrame);

  Position positionFootAtLiftOffToDesiredFootHoldInWorldFrame = getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame(*leg);
  positionWorldToFootHoldInWorldFrame_[leg->getId()] = positionWorldToFootOnTerrainAtLiftOffInWorldFrame
                                                      + positionFootAtLiftOffToDesiredFootHoldInWorldFrame;

  Position positionWorldToFootHoldInWorldFrame = positionWorldToFootHoldInWorldFrame_[leg->getId()];
  validateFootHold(positionWorldToFootHoldInWorldFrame);
  Position validatedPositionFootAtLiftOffToDesiredFootHoldInWorldFrame = positionWorldToFootHoldInWorldFrame
                                                                         - positionWorldToFootOnTerrainAtLiftOffInWorldFrame;

  double interpolationParameter = getInterpolationPhase(*leg);
  Position positionFootOnTerrainAtLiftOffToDesiredFootOnTerrainInWorldFrame = Position(
                                                                                        // x
                                                                                        getHeadingComponentOfFootStep(interpolationParameter,
                                                                                        0.0,
                                                                                        validatedPositionFootAtLiftOffToDesiredFootHoldInWorldFrame.x(),
                                                                                        const_cast<LegBase*>(leg)),
                                                                                        // y
                                                                                        getLateralComponentOfFootStep(interpolationParameter,
                                                                                        0.0,
                                                                                        validatedPositionFootAtLiftOffToDesiredFootHoldInWorldFrame.y(),
                                                                                        const_cast<LegBase*>(leg)),
                                                                                        // z
                                                                                        0.0
                                                                                        );

  Position positionDesiredFootOnTerrainToDesiredFootInControlFrame =  getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(*leg,
                                                                                                       orientationWorldToControl.rotate(positionFootOnTerrainAtLiftOffToDesiredFootOnTerrainInWorldFrame) ); // z
  Position positionWorldToDesiredFootInWorldFrame = positionWorldToFootOnTerrainAtLiftOffInWorldFrame
                                                    + positionFootOnTerrainAtLiftOffToDesiredFootOnTerrainInWorldFrame
                                                    + orientationWorldToControl.inverseRotate(positionDesiredFootOnTerrainToDesiredFootInControlFrame);
  return positionWorldToDesiredFootInWorldFrame;
  //---
}


void FootPlacementStrategyStaticGait::validateFootHold(Position& positionWorldToDesiredFootHoldInWorldFrame) {
  // todo: check if foot hold is feasible
}


// Evaluate feed forward component
Position FootPlacementStrategyStaticGait::getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame(const LegBase& leg)   {
  double stanceDuration = leg.getStanceDuration();

  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
//  Position positionWorldToHipOnTerrainInWorldFrame = leg.getPositionWorldToHipInWorldFrame();
//  terrain_->getHeight(positionWorldToHipOnTerrainInWorldFrame);

  Position positionWorldToHipOnTerrainInWorldFrame;
  switch (leg.getId()) {
    case (0):
    positionWorldToHipOnTerrainInWorldFrame = Position(0.25, 0.18, 0.0);
        break;
    case (1):
    positionWorldToHipOnTerrainInWorldFrame = Position(0.25, -0.18, 0.0);
        break;
    case (2):
    positionWorldToHipOnTerrainInWorldFrame = Position(-0.25, 0.18, 0.0);
        break;
    case (3):
    positionWorldToHipOnTerrainInWorldFrame = Position(-0.25, -0.18, 0.0);
        break;
    default: break;
  }

  Position positionWorldToBaseOnTerrainInWorldFrame = Position();
  for (auto legAuto: *legs_) {
    positionWorldToBaseOnTerrainInWorldFrame += legAuto->getPositionWorldToFootInWorldFrame()/legs_->size();
  }
  terrain_->getHeight(positionWorldToBaseOnTerrainInWorldFrame);
  positionWorldToHipOnTerrainInWorldFrame += positionWorldToBaseOnTerrainInWorldFrame;

  Position positionFootAtLiftOffToHipOnTerrainInWorldFrame = positionWorldToHipOnTerrainInWorldFrame
                                                             - leg.getStateLiftOff().getPositionWorldToFootInWorldFrame();
  Position positionFootAtLiftOffToHipOnTerrainInControlFrame = orientationWorldToControl.rotate(positionFootAtLiftOffToHipOnTerrainInWorldFrame);

  Position headingAxisOfControlFrame = Position::UnitX();
  Position positionDesiredFootOnTerrainVelocityHeadingOffsetInControlFrame = Position(torso_->getDesiredState().getLinearVelocityBaseInControlFrame().toImplementation().cwiseProduct(headingAxisOfControlFrame.toImplementation()))
                                                                      *stanceDuration*0.5;

  Position lateralAxisOfControlFrame = Position::UnitY();
  Position positionDesiredFootOnTerrainVelocityLateralOffsetInControlFrame = Position(torso_->getDesiredState().getLinearVelocityBaseInControlFrame().toImplementation().cwiseProduct(lateralAxisOfControlFrame.toImplementation()))
                                                                      *stanceDuration*0.5;

  Position positionDesiredFootOnTerrainFeedForwardInControlFrame = positionDesiredFootOnTerrainVelocityHeadingOffsetInControlFrame
                                                                   + positionDesiredFootOnTerrainVelocityLateralOffsetInControlFrame
                                                                   + positionFootAtLiftOffToHipOnTerrainInControlFrame;

  return positionDesiredFootOnTerrainFeedForwardInControlFrame;
}


Position FootPlacementStrategyStaticGait::getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(const LegBase& leg) {
  const RotationQuaternion& orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();

  positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg.getId()] = getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame(leg);
//  positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg.getId()] = getPositionDesiredFootHoldOnTerrainFeedBackInControlFrame(leg);

  Position positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame = positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg.getId()]
                                                                                     /*+ positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg.getId()]*/;

  //--- save for debug
  //Position positionWorldToHipOnPlaneAlongNormalInWorldFrame = getPositionProjectedOnPlaneAlongSurfaceNormal(leg.getWorldToHipPositionInWorldFrame());

  Position positionWorldToHipVerticalOnPlaneInWorldFrame = leg.getStateLiftOff().getPositionWorldToHipInWorldFrame();
//  Position positionWorldToHipVerticalOnPlaneInWorldFrame = leg.getWorldToHipPositionInWorldFrame();
  terrain_->getHeight(positionWorldToHipVerticalOnPlaneInWorldFrame);

  positionWorldToFootHoldInWorldFrame_[leg.getId()] = positionWorldToHipVerticalOnPlaneInWorldFrame
                                                      + orientationWorldToControl.inverseRotate(positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame);

//  std::cout << "foothold components for leg: " << leg.getId() << std::endl
//            << "world to hip on t: " << positionWorldToHipVerticalOnPlaneInWorldFrame << std::endl
//            << "hip on t to foot: " << orientationWorldToControl.inverseRotate(positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame) << std::endl;
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

