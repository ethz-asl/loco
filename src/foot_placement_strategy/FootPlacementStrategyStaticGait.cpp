/*
 * FootPlacementStrategyStaticGait.cpp
 *
 *  Created on: Oct 6, 2014
 *      Author: dario
 */


#include "loco/foot_placement_strategy/FootPlacementStrategyStaticGait.hpp"

namespace loco {


FootPlacementStrategyStaticGait::FootPlacementStrategyStaticGait(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain) :
    FootPlacementStrategyFreePlane(legs, torso, terrain),
    positionBaseOnTerrainToDefaultFootInControlFrame_(legs_->size())
{

  stepInterpolationFunction_.clear();
  stepInterpolationFunction_.addKnot(0, 0);
  stepInterpolationFunction_.addKnot(1.0, 1);

  stepFeedbackScale_ = 0.0;

  positionWorldToCenterOfFeetAtLiftOffInWorldFrame_ = Position();

  positionBaseOnTerrainToDefaultFootInControlFrame_[0] = Position(0.25, 0.18, 0.0);
  positionBaseOnTerrainToDefaultFootInControlFrame_[1] = Position(0.25, -0.18, 0.0);
  positionBaseOnTerrainToDefaultFootInControlFrame_[2] = Position(-0.25, 0.18, 0.0);
  positionBaseOnTerrainToDefaultFootInControlFrame_[3] = Position(-0.25, -0.18, 0.0);

}


FootPlacementStrategyStaticGait::~FootPlacementStrategyStaticGait() {

}

/*
 * Foot holds are evaluated with respect to the foot positions at liftoff.
 *
 */
Position FootPlacementStrategyStaticGait::getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep) {
  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  Position positionWorldToFootOnTerrainAtLiftOffInWorldFrame = leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame();
  terrain_->getHeight(positionWorldToFootOnTerrainAtLiftOffInWorldFrame);

  Position positionFootAtLiftOffToDesiredFootHoldInWorldFrame = orientationWorldToControl.inverseRotate(getPositionFootAtLiftOffToDesiredFootHoldInControlFrame(*leg));

  // x-y offset
  Position positionWorldToFootHoldInWorldFrame = positionWorldToFootOnTerrainAtLiftOffInWorldFrame
                                                 + positionFootAtLiftOffToDesiredFootHoldInWorldFrame;

  // orientation offset
  positionWorldToFootHoldInWorldFrame_[leg->getId()] = getPositionDesiredFootHoldOrientationOffsetInWorldFrame(*leg, positionWorldToFootHoldInWorldFrame);

  positionWorldToFootHoldInWorldFrame = positionWorldToFootHoldInWorldFrame_[leg->getId()];
  validateFootHold(positionWorldToFootHoldInWorldFrame);
  Position validatedPositionFootAtLiftOffToDesiredFootHoldInWorldFrame = positionWorldToFootHoldInWorldFrame
                                                                         - positionWorldToFootOnTerrainAtLiftOffInWorldFrame;

  /*
   * Interpolate on the x-y plane
   */
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

  /*
   * Interpolate height trajectory
   */
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


Position FootPlacementStrategyStaticGait::getPositionDesiredFootHoldOrientationOffsetInWorldFrame(const LegBase& leg,
                                                                                                  const Position& positionWorldToDesiredFootHoldBeforeOrientationOffsetInWorldFrame) {
  // rotational component
  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  LocalAngularVelocity desiredAngularVelocityInControlFrame = torso_->getDesiredState().getAngularVelocityBaseInControlFrame();
  LocalAngularVelocity desiredAngularVelocityInWorldFrame = orientationWorldToControl.inverseRotate(desiredAngularVelocityInControlFrame);

  Position rho = positionWorldToDesiredFootHoldBeforeOrientationOffsetInWorldFrame - positionWorldToCenterOfFeetAtLiftOffInWorldFrame_;

  return positionWorldToDesiredFootHoldBeforeOrientationOffsetInWorldFrame
         + Position( desiredAngularVelocityInWorldFrame.toImplementation().cross( rho.toImplementation() ) );
}


// Evaluate feed forward component
Position FootPlacementStrategyStaticGait::getPositionFootAtLiftOffToDesiredFootHoldInControlFrame(const LegBase& leg)   {
  double stanceDuration = leg.getStanceDuration();

  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  Position positionCenterOfFeetAtLiftOffToDefaultFootInWorldFrame = orientationWorldToControl.inverseRotate(positionBaseOnTerrainToDefaultFootInControlFrame_[leg.getId()]);

  /* Update center of feet at lift off */
  Position positionWorldToCenterOfFeetAtLiftOffInWorldFrame = Position();
  for (auto legAuto: *legs_) {
    positionWorldToCenterOfFeetAtLiftOffInWorldFrame += legAuto->getStateLiftOff()->getPositionWorldToFootInWorldFrame()/legs_->size();
  }
  terrain_->getHeight(positionWorldToCenterOfFeetAtLiftOffInWorldFrame);
  positionWorldToCenterOfFeetAtLiftOffInWorldFrame_ = positionWorldToCenterOfFeetAtLiftOffInWorldFrame;

  Position positionWorldToDefafultFootInWorldFrame = positionCenterOfFeetAtLiftOffToDefaultFootInWorldFrame
                                                    + positionWorldToCenterOfFeetAtLiftOffInWorldFrame;

  Position positionFootAtLiftOffToDefaultFootInWorldFrame = positionWorldToDefafultFootInWorldFrame
                                                            - leg.getStateLiftOff().getPositionWorldToFootInWorldFrame();
  Position positionFootAtLiftOffToDefaultFootInControlFrame = orientationWorldToControl.rotate(positionFootAtLiftOffToDefaultFootInWorldFrame);

  // heading component
  Position headingAxisOfControlFrame = Position::UnitX();
  Position positionDesiredFootOnTerrainVelocityHeadingOffsetInControlFrame = Position(torso_->getDesiredState().getLinearVelocityBaseInControlFrame().toImplementation().cwiseProduct(headingAxisOfControlFrame.toImplementation()))
                                                                      *stanceDuration*0.5;

  // lateral component
  Position lateralAxisOfControlFrame = Position::UnitY();
  Position positionDesiredFootOnTerrainVelocityLateralOffsetInControlFrame = Position(torso_->getDesiredState().getLinearVelocityBaseInControlFrame().toImplementation().cwiseProduct(lateralAxisOfControlFrame.toImplementation()))
                                                                      *stanceDuration*0.5;

  // build the desired foot step displacement
  Position positionFootAtLiftOffToDesiredFootHoldInControlFrame = positionFootAtLiftOffToDefaultFootInControlFrame                    // default position
                                                                  + positionDesiredFootOnTerrainVelocityHeadingOffsetInControlFrame   // heading
                                                                  + positionDesiredFootOnTerrainVelocityLateralOffsetInControlFrame;  // lateral

  return positionFootAtLiftOffToDesiredFootHoldInControlFrame;
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

