/*
 * FootPlacementStrategyFreePlane.cpp
 *
 *  Created on: Sep 16, 2014
 *      Author: dario
 */

#include "loco/foot_placement_strategy/FootPlacementStrategyFreePlane.hpp"
#include "loco/common/TorsoBase.hpp"
#include "loco/temp_helpers/math.hpp"

namespace loco {

FootPlacementStrategyFreePlane::FootPlacementStrategyFreePlane(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain) :
    FootPlacementStrategyInvertedPendulum(legs, torso, terrain)
{

  for (auto leg : *legs_) {
    positionWorldToHipOnPlaneAlongNormalInWorldFrame_[leg->getId()].setZero();
    positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame_[leg->getId()].setZero();
    positionDesiredFootOnTerrainToDesiredFootInWorldFrame_[leg->getId()].setZero();
    positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg->getId()].setZero();
    positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg->getId()].setZero();
    positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()].setZero();
  }

}


FootPlacementStrategyFreePlane::~FootPlacementStrategyFreePlane() {

}

bool FootPlacementStrategyFreePlane::initialize(double dt) {
  for (auto leg : *legs_) {
    // save the hip position at lift off for trajectory generation
    Position positionWorldToHipAtLiftOffInWorldFrame = leg->getStateLiftOff()->getHipPositionInWorldFrame();
    positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()] = getPositionProjectedOnPlaneAlongSurfaceNormal(positionWorldToHipAtLiftOffInWorldFrame);
    Position positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame = positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()];
    leg->getStateLiftOff()->setPositionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame(positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame);

  }
  return true;
}


void FootPlacementStrategyFreePlane::advance(double dt) {

//    for (auto leg : *legs_) {
//      if (!leg->isSupportLeg()) {
//        Position positionWorldToFootInWorldFrame = getDesiredWorldToFootPositionInWorldFrame(leg, 0.0);
//
//        if (!getBestFootholdsFromCurrentFootholdInWorldFrame(positionWorldToFootInWorldFrame)) {
//          // TODO: handle exception
//        }
//
//        leg->setDesireWorldToFootPositionInWorldFrame(positionWorldToFootInWorldFrame); // for debugging
//        const Position positionBaseToFootInWorldFrame = positionWorldToFootInWorldFrame - torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
//        const Position positionBaseToFootInBaseFrame  = torso_->getMeasuredState().getOrientationWorldToBase().rotate(positionBaseToFootInWorldFrame);
//        leg->setDesiredJointPositions(leg->getJointPositionsFromBaseToFootPositionInBaseFrame(positionBaseToFootInBaseFrame));
//      }
//    }


  for (auto leg : *legs_) {

    // save the hip position at lift off for trajectory generation
    if (leg->getStateLiftOff()->isNow()) {
      Position positionWorldToHipAtLiftOffInWorldFrame = leg->getStateLiftOff()->getHipPositionInWorldFrame();
            positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()] = getPositionProjectedOnPlaneAlongSurfaceNormal(positionWorldToHipAtLiftOffInWorldFrame);
            Position positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame = positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()];
            leg->getStateLiftOff()->setPositionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame(positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame);
    }

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


Position FootPlacementStrategyFreePlane::getPositionProjectedOnPlaneAlongSurfaceNormal(const Position& position) {
  // For theory background see Papula, "Mathematische Formelasammlung", pag. 62
  Vector n;
  terrain_->getNormal(position, n);
  double terrainHeightAtPosition;
  terrain_->getHeight(position, terrainHeightAtPosition);

  Position r1, rq;
  r1 = Position::Zero();
  terrain_->getHeight(r1);
  rq = position;
  Position r1q = rq-r1;

  double distanceFromSurfaceAlongSurfaceNormal = fabs(n.dot((Vector)r1q))/n.norm();

  if (position.z() > terrainHeightAtPosition) {
    return (position - distanceFromSurfaceAlongSurfaceNormal*Position(n));
  }
  else if (position.z() < terrainHeightAtPosition) {
    return (position + distanceFromSurfaceAlongSurfaceNormal*Position(n));
  }
  else {
    return position;
  }
}


Position FootPlacementStrategyFreePlane::getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep) {

  //--- Setup containers
  Position positionWorldToDesiredFootInWorldFrame;
  Position positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame;
  Position positionDesiredFootOnTerrainToDesiredFootInControlFrame;
  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  //---

  // Project hip position on the plane along the plane normal direction
  Position positionWorldToHipOnPlaneAlongNormalInWorldFrame = getPositionProjectedOnPlaneAlongSurfaceNormal(leg->getWorldToHipPositionInWorldFrame());

  //--- Evaluate the interpolation contributions
  positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame = getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(*leg); //x-y
  positionDesiredFootOnTerrainToDesiredFootInControlFrame = getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(*leg,
                                                                                                                       positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame); // z
  //---

  //--- Build the world to desired foot position and return it
  positionWorldToDesiredFootInWorldFrame = positionWorldToHipOnPlaneAlongNormalInWorldFrame
                                           + orientationWorldToControl.inverseRotate(positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame) // x-y
                                           + orientationWorldToControl.inverseRotate(positionDesiredFootOnTerrainToDesiredFootInControlFrame); // z

  //--- Save for debugging
  positionWorldToHipOnPlaneAlongNormalInWorldFrame_[leg->getId()] = positionWorldToHipOnPlaneAlongNormalInWorldFrame;
  positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame_[leg->getId()] = orientationWorldToControl.inverseRotate(positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame);
  positionDesiredFootOnTerrainToDesiredFootInWorldFrame_[leg->getId()] = orientationWorldToControl.inverseRotate(positionDesiredFootOnTerrainToDesiredFootInControlFrame);
  //---

  return positionWorldToDesiredFootInWorldFrame;
  //---

}


// Interpolate height
Position FootPlacementStrategyFreePlane::getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(const LegBase& leg, const Position& positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame)  {
  Position positionDesiredFootOnTerrainToDesiredFootInControlFrame;
  Vector normalToPlaneAtCurrentFootPositionInWorldFrame, normalToPlaneAtCurrentFootPositionInControlFrame;

  const double interpolationParameter = getInterpolationPhase(leg);
  const double desiredFootHeight = const_cast<SwingFootHeightTrajectory*>(&swingFootHeightTrajectory_)->evaluate(interpolationParameter);

  terrain_->getNormal(positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame,
                      normalToPlaneAtCurrentFootPositionInWorldFrame);

  normalToPlaneAtCurrentFootPositionInControlFrame = torso_->getMeasuredState().getOrientationWorldToControl().rotate(normalToPlaneAtCurrentFootPositionInWorldFrame);

  positionDesiredFootOnTerrainToDesiredFootInControlFrame = desiredFootHeight*Position(normalToPlaneAtCurrentFootPositionInControlFrame);
  return positionDesiredFootOnTerrainToDesiredFootInControlFrame;
}


Position FootPlacementStrategyFreePlane::getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame(const LegBase& leg)   {
  Position positionDesiredFootOnTerrainDefaultOffsetInControlFrame;
  Position positionDesiredFootOnTerrainVelocityOffsetInControlFrame;
  Position positionDesiredFootOnTerrainFeedForwardInControlFrame;

  double stanceDuration = leg.getStanceDuration();

  positionDesiredFootOnTerrainDefaultOffsetInControlFrame = leg.getProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame();

  Position headingAxisOfControlFrame = Position::UnitX();
  positionDesiredFootOnTerrainVelocityOffsetInControlFrame = Position(torso_->getDesiredState().getLinearVelocityBaseInControlFrame().toImplementation().cwiseProduct(headingAxisOfControlFrame.toImplementation()))
                                                             *stanceDuration*0.5;

  positionDesiredFootOnTerrainFeedForwardInControlFrame = positionDesiredFootOnTerrainDefaultOffsetInControlFrame
                                                          + positionDesiredFootOnTerrainVelocityOffsetInControlFrame;
  return positionDesiredFootOnTerrainFeedForwardInControlFrame;
}


Position FootPlacementStrategyFreePlane::getPositionDesiredFootHoldOnTerrainFeedBackInControlFrame(const LegBase& leg) {
  //---
  Position positionDesiredFootHoldOnTerrainFeedBackInControlFrame;
  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  RotationQuaternion orientationWorldToBase = torso_->getMeasuredState().getOrientationWorldToBase();
  LinearVelocity linearVelocityErrorInWorldFrame;
  LinearVelocity linearVeloctyReferenceInWorldFrame;
  LinearVelocity linearVeloctyDesiredInWorldFrame;
  LinearVelocity linearVeloctyDesiredInControlFrame;
  LinearVelocity linearVeloctyDesiredHeadingInControlFrame;
  //---

  //--- Get desired velocity heading component in control frame
  Vector axisXOfControlFrame = Vector::UnitX();
  linearVeloctyDesiredInControlFrame = torso_->getDesiredState().getLinearVelocityBaseInControlFrame();
  linearVeloctyDesiredHeadingInControlFrame = LinearVelocity(linearVeloctyDesiredInControlFrame.toImplementation().cwiseProduct(axisXOfControlFrame.toImplementation()));
  linearVeloctyDesiredInWorldFrame = orientationWorldToControl.inverseRotate(linearVeloctyDesiredHeadingInControlFrame);
  //---

  //--- Get reference velocity estimation
  linearVeloctyReferenceInWorldFrame = (leg.getHipLinearVelocityInWorldFrame()
                                        + orientationWorldToBase.inverseRotate(torso_->getMeasuredState().getLinearVelocityBaseInBaseFrame())
                                        )/2.0;
  //---


  linearVelocityErrorInWorldFrame = linearVeloctyReferenceInWorldFrame
                                    - linearVeloctyDesiredInWorldFrame;

  //--- Get inverted pendulum height
  double terrainHeightAtHipInWorldFrame;
  terrain_->getHeight(leg.getWorldToHipPositionInWorldFrame(), terrainHeightAtHipInWorldFrame);
  const double heightInvertedPendulum = fabs(leg.getWorldToHipPositionInWorldFrame().z() - terrainHeightAtHipInWorldFrame);
  //---

  const double gravitationalAccleration = torso_->getProperties().getGravity().norm();
  const Position positionDesiredFootHoldOnTerrainFeedBackInWorldFrame = Position(linearVelocityErrorInWorldFrame
                                                                             *(stepFeedbackScale_*1.0)
                                                                             *std::sqrt(heightInvertedPendulum/gravitationalAccleration));

  positionDesiredFootHoldOnTerrainFeedBackInControlFrame = orientationWorldToControl.rotate(positionDesiredFootHoldOnTerrainFeedBackInWorldFrame);
  positionDesiredFootHoldOnTerrainFeedBackInControlFrame.z() = 0.0; // project on terrain
  return positionDesiredFootHoldOnTerrainFeedBackInControlFrame;

}


double FootPlacementStrategyFreePlane::getInterpolationPhase(const LegBase& leg)  {
  double swingPhase = 1;
  if (!leg.isSupportLeg()) {
    swingPhase = leg.getSwingPhase();
  }
  return swingPhase;
}


Position FootPlacementStrategyFreePlane::getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(const LegBase& leg) {
  Position positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame;

  const RotationQuaternion& orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();

  positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg.getId()] = getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame(leg);
  positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg.getId()] = getPositionDesiredFootHoldOnTerrainFeedBackInControlFrame(leg);

  Position positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame = positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg.getId()]
                                                                                     + positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg.getId()];

  const Position positionHipOnTerrainAlongNormalToFootAtLiftOffInWorldFrame = leg.getStateLiftOff().getFootPositionInWorldFrame()
                                                                              -leg.getStateLiftOff().getPositionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame();

  const Position positionHipOnTerrainAlongNormalToFootAtLiftOffInControlFrame = orientationWorldToControl.rotate(positionHipOnTerrainAlongNormalToFootAtLiftOffInWorldFrame);

  double interpolationParameter = getInterpolationPhase(leg);

  positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame = Position(
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


bool FootPlacementStrategyFreePlane::getBestFootholdsFromCurrentFootholdInWorldFrame(loco::Position& positionWorldToFootInWorldFrame) {
  return true;
}


} /* namespace loco */
