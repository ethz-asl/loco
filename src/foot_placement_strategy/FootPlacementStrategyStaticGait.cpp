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
    positionBaseOnTerrainToDefaultFootInControlFrame_(legs_->size()),
    positionWorldToValidatedDesiredFootHoldInWorldFrame_(legs_->size()),
    comControl_(nullptr),
    newFootHolds_(legs_->size())
{

  stepInterpolationFunction_.clear();
  stepInterpolationFunction_.addKnot(0, 0);
  stepInterpolationFunction_.addKnot(1.0, 1);

  stepFeedbackScale_ = 0.0;

  positionWorldToCenterOfFeetAtLiftOffInWorldFrame_ = Position();

//  positionBaseOnTerrainToDefaultFootInControlFrame_[0] = Position(0.25, 0.18, 0.0);
//  positionBaseOnTerrainToDefaultFootInControlFrame_[1] = Position(0.25, -0.18, 0.0);
//  positionBaseOnTerrainToDefaultFootInControlFrame_[2] = Position(-0.25, 0.18, 0.0);
//  positionBaseOnTerrainToDefaultFootInControlFrame_[3] = Position(-0.25, -0.18, 0.0);

  for (auto leg: *legs_) {
    positionBaseOnTerrainToDefaultFootInControlFrame_[leg->getId()].setZero();
    positionWorldToValidatedDesiredFootHoldInWorldFrame_[leg->getId()].setZero();
    positionWorldToFootHoldInWorldFrame_[leg->getId()].setZero();
    newFootHolds_[leg->getId()].setZero();
  }

}


FootPlacementStrategyStaticGait::~FootPlacementStrategyStaticGait() {

}


void FootPlacementStrategyStaticGait::setCoMControl(CoMOverSupportPolygonControlBase* comControl) {
  comControl_ = static_cast<CoMOverSupportPolygonControlStaticGait*>(comControl);
}


bool FootPlacementStrategyStaticGait::initialize(double dt) {
  FootPlacementStrategyFreePlane::initialize(dt);

  for (auto leg: *legs_) {
    positionWorldToValidatedDesiredFootHoldInWorldFrame_[leg->getId()] = leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame();
    positionBaseOnTerrainToDefaultFootInControlFrame_[leg->getId()] = leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame();
    terrain_->getHeight(positionBaseOnTerrainToDefaultFootInControlFrame_[leg->getId()]);
    positionWorldToCenterOfFeetAtLiftOffInWorldFrame_ += positionWorldToValidatedDesiredFootHoldInWorldFrame_[leg->getId()]/legs_->size();
    positionWorldToFootHoldInWorldFrame_[leg->getId()] = positionBaseOnTerrainToDefaultFootInControlFrame_[leg->getId()];
    newFootHolds_[leg->getId()] = positionWorldToValidatedDesiredFootHoldInWorldFrame_[leg->getId()];
  }

  return true;
}


bool FootPlacementStrategyStaticGait::advance(double dt) {

  for (auto leg : *legs_) {
    // save the hip position at lift off for trajectory generation
    if (leg->shouldBeGrounded() ||
        (!leg->shouldBeGrounded() && leg->isGrounded() && leg->getSwingPhase() < 0.25)
    ) {
      Position positionWorldToHipAtLiftOffInWorldFrame = leg->getPositionWorldToHipInWorldFrame();
      positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()] = getPositionProjectedOnPlaneAlongSurfaceNormal(positionWorldToHipAtLiftOffInWorldFrame);
      Position positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame = positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()];

      Position positionWorldToHipOnTerrainAlongWorldZInWorldFrame = positionWorldToHipAtLiftOffInWorldFrame;
      terrain_->getHeight(positionWorldToHipOnTerrainAlongWorldZInWorldFrame);

      /*
       * WARNING: these were also updated by the event detector
       */
      leg->getStateLiftOff()->setPositionWorldToHipOnTerrainAlongWorldZInWorldFrame(positionWorldToHipOnTerrainAlongWorldZInWorldFrame);
      leg->getStateLiftOff()->setPositionWorldToFootInWorldFrame(leg->getPositionWorldToFootInWorldFrame());
      leg->getStateLiftOff()->setPositionWorldToHipInWorldFrame(leg->getPositionWorldToHipInWorldFrame());
      leg->setSwingPhase(leg->getSwingPhase());
    }

    /*
     * Generate a foothold if all feet are grounded
     */
    if (comControl_->getSwingFootChanged()) {
      int swingLeg = comControl_->getNextSwingLeg();
      generateFootHold(legs_->getLegById(swingLeg));
    }


    if (!leg->isSupportLeg()) {
      StateSwitcher* stateSwitcher = leg->getStateSwitcher();

      switch(stateSwitcher->getState()) {
        case(StateSwitcher::States::StanceSlipping):
        case(StateSwitcher::States::StanceLostContact):
          regainContact(leg, dt); break;

        case(StateSwitcher::States::SwingNormal):
        case(StateSwitcher::States::SwingLateLiftOff):
        /*case(StateSwitcher::States::SwingBumpedIntoObstacle):*/ {
          setFootTrajectory(leg);
        }
        break;

        default:
          break;
      }
    }

  }
  return true;
}


void FootPlacementStrategyStaticGait::setFootTrajectory(LegBase* leg) {
    const Position positionWorldToFootInWorldFrame = getDesiredWorldToFootPositionInWorldFrame(leg, 0.0);
    leg->setDesireWorldToFootPositionInWorldFrame(positionWorldToFootInWorldFrame); // for debugging
    const Position positionBaseToFootInWorldFrame = positionWorldToFootInWorldFrame - torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
    const Position positionBaseToFootInBaseFrame  = torso_->getMeasuredState().getOrientationWorldToBase().rotate(positionBaseToFootInWorldFrame);

    leg->setDesiredJointPositions(leg->getJointPositionsFromPositionBaseToFootInBaseFrame(positionBaseToFootInBaseFrame));
}


void FootPlacementStrategyStaticGait::regainContact(LegBase* leg, double dt) {
  Position positionWorldToFootInWorldFrame =  leg->getPositionWorldToFootInWorldFrame();
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
  leg->setDesiredJointPositions(leg->getJointPositionsFromPositionBaseToFootInBaseFrame(positionBaseToFootInBaseFrame));
}


Position FootPlacementStrategyStaticGait::generateFootHold(LegBase* leg) {
  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  Position positionWorldToFootOnTerrainAtLiftOffInWorldFrame = leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame();
  terrain_->getHeight(positionWorldToFootOnTerrainAtLiftOffInWorldFrame);

  Position positionFootAtLiftOffToDesiredFootHoldInWorldFrame = orientationWorldToControl.inverseRotate(getPositionFootAtLiftOffToDesiredFootHoldInControlFrame(*leg));

  // x-y offset
  Position positionWorldToFootHoldInWorldFrame = positionWorldToFootOnTerrainAtLiftOffInWorldFrame
                                                 + positionFootAtLiftOffToDesiredFootHoldInWorldFrame;

  // orientation offset - rotate foothold position vector around world z axis according to desired angular velocity
  positionWorldToFootHoldInWorldFrame = getPositionDesiredFootHoldOrientationOffsetInWorldFrame(*leg, positionWorldToFootHoldInWorldFrame);


  // update class member and get correct terrain height at foot hold
  positionWorldToFootHoldInWorldFrame_[leg->getId()] = positionWorldToFootHoldInWorldFrame;
  terrain_->getHeight(positionWorldToFootHoldInWorldFrame_[leg->getId()]);

  return positionWorldToFootHoldInWorldFrame;
}


/*
 * Foot holds are evaluated with respect to the foot positions at liftoff.
 *
 */
Position FootPlacementStrategyStaticGait::getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep) {
  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  Position positionWorldToFootAtLiftOffInWorldFrame = leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame();

  // validate the foot step
  Position positionWorldToFootHoldInWorldFrame = positionWorldToFootHoldInWorldFrame_[leg->getId()];
  Position positionWorldToValidatedFootHoldInWorldFrame = getValidatedFootHold(positionWorldToFootHoldInWorldFrame);
  positionWorldToValidatedDesiredFootHoldInWorldFrame_[leg->getId()] = positionWorldToValidatedFootHoldInWorldFrame;
  Position positionFootAtLiftOffToValidatedDesiredFootHoldInWorldFrame = positionWorldToValidatedFootHoldInWorldFrame
                                                                         - positionWorldToFootAtLiftOffInWorldFrame;

  /*
   * Interpolate on the x-y plane
   */
  double interpolationParameter = getInterpolationPhase(*leg);
  Position positionFootOnTerrainAtLiftOffToDesiredFootOnTerrainInWorldFrame = Position(
                                                                                        // x
                                                                                        getHeadingComponentOfFootStep(interpolationParameter,
                                                                                        0.0,
                                                                                        positionFootAtLiftOffToValidatedDesiredFootHoldInWorldFrame.x(),
                                                                                        const_cast<LegBase*>(leg)),
                                                                                        // y
                                                                                        getLateralComponentOfFootStep(interpolationParameter,
                                                                                        0.0,
                                                                                        positionFootAtLiftOffToValidatedDesiredFootHoldInWorldFrame.y(),
                                                                                        const_cast<LegBase*>(leg)),
                                                                                        // z
                                                                                        0.0
                                                                                        );

  /*
   * Interpolate height trajectory
   */
  Position positionDesiredFootOnTerrainToDesiredFootInControlFrame =  getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(*leg,
                                                                                                       orientationWorldToControl.rotate(positionFootOnTerrainAtLiftOffToDesiredFootOnTerrainInWorldFrame) ); // z


  Position positionWorldToDesiredFootInWorldFrame = positionWorldToFootAtLiftOffInWorldFrame
                                                    + positionFootOnTerrainAtLiftOffToDesiredFootOnTerrainInWorldFrame
                                                    + orientationWorldToControl.inverseRotate(positionDesiredFootOnTerrainToDesiredFootInControlFrame);
  return positionWorldToDesiredFootInWorldFrame;
  //---
}


Position FootPlacementStrategyStaticGait::getValidatedFootHold(const Position& positionWorldToDesiredFootHoldInWorldFrame) {
  // todo: check if foot hold is feasible
  Position validated = positionWorldToDesiredFootHoldInWorldFrame;
  return validated;

}


Position FootPlacementStrategyStaticGait::getPositionWorldToValidatedDesiredFootHoldInWorldFrame(int legId) const {
  // todo: check if legId is in admissible range
  return positionWorldToValidatedDesiredFootHoldInWorldFrame_[legId];
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
    //positionWorldToCenterOfFeetAtLiftOffInWorldFrame += legAuto->getStateLiftOff()->getPositionWorldToFootInWorldFrame()/legs_->size();
    positionWorldToCenterOfFeetAtLiftOffInWorldFrame += positionWorldToValidatedDesiredFootHoldInWorldFrame_[legAuto->getId()]/legs_->size();
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

