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


Position FootPlacementStrategyStaticGait::getPositionDesiredFootHoldOrientationOffsetInControlFrame(const LegBase& leg) {
  return Position();
}


// Evaluate feed forward component
Position FootPlacementStrategyStaticGait::getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame(const LegBase& leg)   {
  double stanceDuration = leg.getStanceDuration();

  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
//  Position positionWorldToHipOnTerrainInWorldFrame = leg.getPositionWorldToHipInWorldFrame();
//  terrain_->getHeight(positionWorldToHipOnTerrainInWorldFrame);

  Position rho = Position();
  double dt = leg.getStanceDuration()+leg.getSwingDuration();
  LocalAngularVelocity desiredAngularVelocityInControlFrame = torso_->getDesiredState().getAngularVelocityBaseInControlFrame();
  RotationQuaternion rotation = RotationQuaternion();
  double angle = 0.0;

  Position positionWorldToHipOnTerrainInWorldFrame;
  switch (leg.getId()) {
    case (0): {
          rho = Position(0.25, 0.18, 0.0);

//          // rotational component
//          angle = desiredAngularVelocityInControlFrame.z()*dt;
//          rotation = RotationQuaternion(AngleAxis(angle, 0.0, 0.0, 1.0));
//          rho = rotation.inverseRotate(rho);

          positionWorldToHipOnTerrainInWorldFrame = rho;
        }
        break;
    case (1): {
          rho = Position(0.25, -0.18, 0.0);

//          // rotational component
//          angle = desiredAngularVelocityInControlFrame.z()*dt;
//          rotation = RotationQuaternion(AngleAxis(angle, 0.0, 0.0, 1.0));
//          rho = rotation.inverseRotate(rho);

          positionWorldToHipOnTerrainInWorldFrame = rho;
        }
        break;
    case (2):{
          rho = Position(-0.25, 0.18, 0.0);

//          // rotational component
//          angle = desiredAngularVelocityInControlFrame.z()*dt;
//          rotation = RotationQuaternion(AngleAxis(angle, 0.0, 0.0, 1.0));
//          rho = rotation.inverseRotate(rho);

          positionWorldToHipOnTerrainInWorldFrame = rho;
        }
        break;
    case (3):{
          rho = Position(-0.25, -0.18, 0.0);

//          // rotational component
//          angle = desiredAngularVelocityInControlFrame.z()*dt;
//          rotation = RotationQuaternion(AngleAxis(angle, 0.0, 0.0, 1.0));
//          rho = rotation.inverseRotate(rho);

          positionWorldToHipOnTerrainInWorldFrame = rho;
        }
        break;
    default: break;
  }

  Position positionWorldToBaseOnTerrainInWorldFrame = Position();
  for (auto legAuto: *legs_) {
    positionWorldToBaseOnTerrainInWorldFrame += legAuto->getStateLiftOff()->getPositionWorldToFootInWorldFrame()/legs_->size();
  }
  terrain_->getHeight(positionWorldToBaseOnTerrainInWorldFrame);
  positionWorldToHipOnTerrainInWorldFrame += positionWorldToBaseOnTerrainInWorldFrame;

  Position positionFootAtLiftOffToHipOnTerrainInWorldFrame = positionWorldToHipOnTerrainInWorldFrame
                                                             - leg.getStateLiftOff().getPositionWorldToFootInWorldFrame();
  Position positionFootAtLiftOffToHipOnTerrainInControlFrame = orientationWorldToControl.rotate(positionFootAtLiftOffToHipOnTerrainInWorldFrame);

  // heading component
  Position headingAxisOfControlFrame = Position::UnitX();
  Position positionDesiredFootOnTerrainVelocityHeadingOffsetInControlFrame = Position(torso_->getDesiredState().getLinearVelocityBaseInControlFrame().toImplementation().cwiseProduct(headingAxisOfControlFrame.toImplementation()))
                                                                      *stanceDuration*0.5;

  // lateral component
  Position lateralAxisOfControlFrame = Position::UnitY();
  Position positionDesiredFootOnTerrainVelocityLateralOffsetInControlFrame = Position(torso_->getDesiredState().getLinearVelocityBaseInControlFrame().toImplementation().cwiseProduct(lateralAxisOfControlFrame.toImplementation()))
                                                                      *stanceDuration*0.5;

  // rotational component
  rho = leg.getPositionWorldToFootInWorldFrame() - torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
  terrain_->getHeight(rho);

  LocalAngularVelocity desiredAngularVelocityInWorldFrame = orientationWorldToControl.inverseRotate(desiredAngularVelocityInControlFrame);

  Position positionDesiredFootOnTerrainVelocityRotationalOffsetInControlFrame = orientationWorldToControl.rotate( Position(desiredAngularVelocityInWorldFrame.toImplementation().cross(rho.toImplementation())) );


  // build the desired foot step displacement
  Position positionDesiredFootOnTerrainFeedForwardInControlFrame = positionFootAtLiftOffToHipOnTerrainInControlFrame                      // default position
                                                                   + positionDesiredFootOnTerrainVelocityHeadingOffsetInControlFrame      // heading
                                                                   + positionDesiredFootOnTerrainVelocityLateralOffsetInControlFrame      // lateral
                                                                   /*+ positionDesiredFootOnTerrainVelocityRotationalOffsetInControlFrame*/;  // rotational


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

