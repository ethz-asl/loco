/*****************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Christian Gehring, C. Dario Bellicoso, Péter Fankhauser, Stelian Coros
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*!
* @file     FootPlacementStrategyFreePlane.hpp
* @author   C. Dario Bellicoso, Christian Gehring
* @date     Sep 16, 2014
* @brief
*/

#include "loco/foot_placement_strategy/FootPlacementStrategyFreePlane.hpp"
#include "loco/temp_helpers/math.hpp"

#include "loco/state_switcher/StateSwitcher.hpp"
#include "robotUtils/loggers/logger.hpp"

namespace loco {

FootPlacementStrategyFreePlane::FootPlacementStrategyFreePlane(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain) :
    FootPlacementStrategyInvertedPendulum(legs, torso, terrain),
    telescopicLeverConfiguration_(0.0)
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

  FootPlacementStrategyInvertedPendulum::initialize(dt);

  telescopicLeverConfiguration_ = 0.0;

  for (auto leg : *legs_) {
    // save the hip position at lift off for trajectory generation
    Position positionWorldToHipAtLiftOffInWorldFrame = leg->getStateLiftOff()->getPositionWorldToHipInWorldFrame();
    positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()] = getPositionProjectedOnPlaneAlongSurfaceNormal(positionWorldToHipAtLiftOffInWorldFrame);
    Position positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame = positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()];
    leg->getStateLiftOff()->setPositionWorldToHipOnTerrainAlongWorldZInWorldFrame(positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame);
  }


  /*
   * Setup logger
   */
  if (isFirstTimeInit_) {

    isFirstTimeInit_ = false;
  }

  return true;
}


bool FootPlacementStrategyFreePlane::advance(double dt) {

  for (auto leg : *legs_) {
    // save the hip position at lift off for trajectory generation
    if (leg->shouldBeGrounded() ||
        (!leg->shouldBeGrounded() && leg->isGrounded() && leg->getSwingPhase() < 0.25)
    ) {
      // Project the hip at lift off along the normal to the plane that models the ground
      Position positionWorldToHipAtLiftOffInWorldFrame = leg->getPositionWorldToHipInWorldFrame();
      positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()] = getPositionProjectedOnPlaneAlongSurfaceNormal(positionWorldToHipAtLiftOffInWorldFrame);
      Position positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame = positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()];

      // Project the hip at lift off along the z axis in world frame
      Position positionWorldToHipOnTerrainAlongWorldZInWorldFrame = positionWorldToHipAtLiftOffInWorldFrame;
      terrain_->getHeight(positionWorldToHipOnTerrainAlongWorldZInWorldFrame);

      /*
       * WARNING: these were also updated by the event detector
       */
      leg->getStateLiftOff()->setPositionWorldToHipOnTerrainAlongWorldZInWorldFrame(positionWorldToHipOnTerrainAlongWorldZInWorldFrame);
      leg->getStateLiftOff()->setPositionWorldToFootInWorldFrame(leg->getPositionWorldToFootInWorldFrame());
      leg->getStateLiftOff()->setPositionWorldToHipInWorldFrame(leg->getPositionWorldToHipInWorldFrame());
//      leg->setSwingPhase(leg->getSwingPhase());
    }

    // Decide what to do based on the current state
    if (!leg->isSupportLeg()) {
      StateSwitcher* stateSwitcher = leg->getStateSwitcher();

      switch(stateSwitcher->getState()) {
        case(StateSwitcher::States::StanceSlipping):
        case(StateSwitcher::States::StanceLostContact):
          regainContact(leg, dt); break;

        case(StateSwitcher::States::SwingNormal):
        case(StateSwitcher::States::SwingLateLiftOff):
//        case(StateSwitcher::States::SwingBumpedIntoObstacle):
          setFootTrajectory(leg); break;

        default:
          break;
      }
    }

  }

  return true;
}


Position FootPlacementStrategyFreePlane::getPositionProjectedOnPlaneAlongSurfaceNormal(const Position& position) {
  return terrain_->getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(position);
}


Position FootPlacementStrategyFreePlane::getPositionVerticalHeightOnTerrainToLeverTelescopicConfigurationInWorldFrame(const LegBase& leg) {
  Position positionVerticalHeightOnTerrainToLeverTelescopicConfigurationInWorldFrame;

  // Project hip on terrain along world frame z axis
  Position positionWorldToHipOnPlaneAlongWorldNormalInWorldFrame = leg.getPositionWorldToHipInWorldFrame();
  terrain_->getHeight(positionWorldToHipOnPlaneAlongWorldNormalInWorldFrame);

  // Project hip on terrain along surface normal
  Position positionWorldToHipOnPlaneAlongSurfaceNormalInWorldFrame = getPositionProjectedOnPlaneAlongSurfaceNormal(leg.getPositionWorldToHipInWorldFrame());

  positionVerticalHeightOnTerrainToLeverTelescopicConfigurationInWorldFrame = (positionWorldToHipOnPlaneAlongSurfaceNormalInWorldFrame
                                                                              - positionWorldToHipOnPlaneAlongWorldNormalInWorldFrame)*telescopicLeverConfiguration_;

  // Return difference times the scaler
  return positionVerticalHeightOnTerrainToLeverTelescopicConfigurationInWorldFrame;
}


Position FootPlacementStrategyFreePlane::getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep) {

  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();

  // Find starting point: hip projected vertically on ground
  Position positionWorldToHipOnPlaneAlongNormalInWorldFrame = leg->getPositionWorldToHipInWorldFrame();
  terrain_->getHeight(positionWorldToHipOnPlaneAlongNormalInWorldFrame);

  // Get offset to change between telescopic and lever configuration
  Position positionVerticalHeightOnTerrainToLeverTelescopicConfigurationInWorldFrame = getPositionVerticalHeightOnTerrainToLeverTelescopicConfigurationInWorldFrame(*leg);

  //--- Evaluate the interpolation contributions
  Position positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame = getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(*leg); //x-y
  Position positionDesiredFootOnTerrainToDesiredFootInControlFrame = getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(*leg,
                                                                                                                       positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame); // z
  //---

  //--- Build the world to desired foot position and return it
  Position positionWorldToDesiredFootInWorldFrame = positionWorldToHipOnPlaneAlongNormalInWorldFrame // starting point, hip projected on the plane along world z axis
                                                    + positionVerticalHeightOnTerrainToLeverTelescopicConfigurationInWorldFrame  // lever || telescopic component
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


Position FootPlacementStrategyFreePlane::getOffsetDesiredFootOnTerrainToCorrectedFootOnTerrainInControlFrame(const LegBase& leg) {

  if (leg.getId() == 0 || leg.getId() == 1) {
    return Position();
  }

// Matlab code
//  % shift y 2
//  % alpha = atan2(hipToHipY/2-abs(rb_onplane_worldvertical(2)), ...
//  %               abs(rb_onplane_worldvertical(1))+abs(rf_lf_controlvertical(1)));
//  % u_vec = rb_onplane_worldvertical-rf_lf_controlvertical;
//  % u_vec = u_vec/norm(u_vec);
//  % u = u_vec*hipToHipX/cos(alpha);
//  %
//  % pos_on_hiptohipy = rf_lf_controlvertical(1:2)+u(1:2);
//  % rf_lh_controlvertical(2) = abs(pos_on_hiptohipy(2));
//  % rf_rh_controlvertical(2) = -abs(pos_on_hiptohipy(2));

  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  Position positionWorldToBaseInWorldFrame = torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
  Position positionWorldToBaseOnTerrainInWorldFrame = positionWorldToBaseInWorldFrame;
  terrain_->getHeight(positionWorldToBaseOnTerrainInWorldFrame);

  Position positionWorldToGeometricCenterInWorldFrame = getPositionProjectedOnPlaneAlongSurfaceNormal(positionWorldToBaseInWorldFrame);

  Position r_com = positionWorldToBaseOnTerrainInWorldFrame
                   - positionWorldToGeometricCenterInWorldFrame;

  Position r_hip;

  if (leg.getId() == 3) {
    r_hip = getPositionProjectedOnPlaneAlongSurfaceNormal(legs_->getLeftForeLeg()->getDesiredWorldToFootPositionInWorldFrame())
            - positionWorldToGeometricCenterInWorldFrame;
  }
  if (leg.getId() == 2) {
    r_hip = getPositionProjectedOnPlaneAlongSurfaceNormal(legs_->getRightForeLeg()->getDesiredWorldToFootPositionInWorldFrame())
                - positionWorldToGeometricCenterInWorldFrame;
  }

  Vector u_des = Vector(r_com - r_hip);
  u_des = u_des.normalize();

  //--- Get geometric dimensions
  double hipToHipX = orientationWorldToControl.inverseRotate(legs_->getLeftForeLeg()->getPositionWorldToHipInWorldFrame()).x()
                     - orientationWorldToControl.inverseRotate(legs_->getLeftHindLeg()->getPositionWorldToHipInWorldFrame()).x();

  double hipToHipY = orientationWorldToControl.inverseRotate(legs_->getLeftForeLeg()->getPositionWorldToHipInWorldFrame()).y()
                       - orientationWorldToControl.inverseRotate(legs_->getRightForeLeg()->getPositionWorldToHipInWorldFrame()).y();
  //---

  double alpha = atan2( hipToHipY/2 - r_com.y(),
                        hipToHipX/2 - r_com.x());

  Position r_lf_rh_star = Position(u_des)*hipToHipX/cos(alpha);

  Position r_leghip = getPositionProjectedOnPlaneAlongSurfaceNormal(leg.getPositionWorldToHipInWorldFrame())
                     - positionWorldToGeometricCenterInWorldFrame;

  Position delta = r_leghip - (r_hip + r_lf_rh_star);

  return delta;
}


/*
 * Interpolate height: get the vector pointing from the current interpolated foot hold to the height of the desired foot based on the interpolation phase
 */
Position FootPlacementStrategyFreePlane::getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(const LegBase& leg, const Position& positionHipOnTerrainToDesiredFootOnTerrainInControlFrame)  {
  const double interpolationParameter = getInterpolationPhase(leg);
  const double desiredFootHeight = const_cast<SwingFootHeightTrajectory*>(&swingFootHeightTrajectory_)->evaluate(interpolationParameter);

  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  Position positionHipOnTerrainToDesiredFootOnTerrainInWorldFrame = orientationWorldToControl.inverseRotate(positionHipOnTerrainToDesiredFootOnTerrainInControlFrame);

  Vector normalToPlaneAtCurrentFootPositionInWorldFrame;
  terrain_->getNormal(positionHipOnTerrainToDesiredFootOnTerrainInWorldFrame,
                      normalToPlaneAtCurrentFootPositionInWorldFrame);

  Vector normalToPlaneAtCurrentFootPositionInControlFrame = orientationWorldToControl.rotate(normalToPlaneAtCurrentFootPositionInWorldFrame);

  Position positionDesiredFootOnTerrainToDesiredFootInControlFrame = desiredFootHeight*Position(normalToPlaneAtCurrentFootPositionInControlFrame);
  return positionDesiredFootOnTerrainToDesiredFootInControlFrame;
}


/*
 * Evaluate ***feed forward component*** and ***default offset***
 */
Position FootPlacementStrategyFreePlane::getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame(const LegBase& leg)   {
  double stanceDuration = leg.getStanceDuration();

  Position positionDesiredFootOnTerrainDefaultOffsetInControlFrame = leg.getProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame();

  Position headingAxisOfControlFrame = Position::UnitX();
  Position positionDesiredFootOnTerrainVelocityOffsetInControlFrame = Position(torso_->getDesiredState().getLinearVelocityBaseInControlFrame().toImplementation().cwiseProduct(headingAxisOfControlFrame.toImplementation()))
                                                                      *stanceDuration*0.5;

  Position positionDesiredFootOnTerrainFeedForwardInControlFrame = positionDesiredFootOnTerrainDefaultOffsetInControlFrame
                                                                   + positionDesiredFootOnTerrainVelocityOffsetInControlFrame;
  return positionDesiredFootOnTerrainFeedForwardInControlFrame;
}


// Evaluate feedback component given by the inverted pendulum model
Position FootPlacementStrategyFreePlane::getPositionDesiredFootHoldOnTerrainFeedBackInControlFrame(const LegBase& leg) {
  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  RotationQuaternion orientationWorldToBase = torso_->getMeasuredState().getOrientationWorldToBase();

  //--- Get desired velocity heading component in control frame
  Vector axisXOfControlFrame = Vector::UnitX();
  LinearVelocity linearVeloctyDesiredInControlFrame = torso_->getDesiredState().getLinearVelocityBaseInControlFrame();
  LinearVelocity linearVeloctyDesiredHeadingInControlFrame = LinearVelocity(linearVeloctyDesiredInControlFrame.toImplementation().cwiseProduct(axisXOfControlFrame.toImplementation()));
  LinearVelocity linearVeloctyDesiredInWorldFrame = orientationWorldToControl.inverseRotate(linearVeloctyDesiredHeadingInControlFrame);
  //---

  //--- Get reference velocity estimation
  LinearVelocity linearVeloctyReferenceInWorldFrame = (leg.getLinearVelocityHipInWorldFrame()
                                                      + orientationWorldToBase.inverseRotate(torso_->getMeasuredState().getLinearVelocityBaseInBaseFrame())
                                                      )/2.0;
  //---

  LinearVelocity linearVelocityErrorInWorldFrame = linearVeloctyReferenceInWorldFrame
                                                   - linearVeloctyDesiredInWorldFrame;

  //--- hack
  //linearVelocityErrorInWorldFrame.z() = 0.0;
  //---

  //--- Get inverted pendulum height
  /**********************************************************
   * Method I: project pendulum along z axis in world frame *
   **********************************************************/
  double terrainHeightAtHipInWorldFrame;
  terrain_->getHeight(leg.getPositionWorldToHipInWorldFrame(), terrainHeightAtHipInWorldFrame);
  const double heightInvertedPendulum = fabs(leg.getPositionWorldToHipInWorldFrame().z() - terrainHeightAtHipInWorldFrame);
  /****************
   * End Method I *
   ****************/

  /*************************************************************
   * Method II: project inverted pendulum along surface normal *
   *************************************************************/
//  TerrainModelFreePlane* terrainFreePlane = dynamic_cast<TerrainModelFreePlane*>(terrain_);
//  const double heightInvertedPendulum = terrainFreePlane->getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(leg.getWorldToHipPositionInWorldFrame());
  /*****************
   * End Method II *
   *****************/

  const double gravitationalAccleration = torso_->getProperties().getGravity().norm();
  Position positionDesiredFootHoldOnTerrainFeedBackInWorldFrame = Position(linearVelocityErrorInWorldFrame
                                                                  *stepFeedbackScale_
                                                                  *std::sqrt(heightInvertedPendulum/gravitationalAccleration));
  //--- hack
  //  positionDesiredFootHoldOnTerrainFeedBackInWorldFrame.z() = 0.0;
  //---

  Position positionDesiredFootHoldOnTerrainFeedBackInControlFrame = orientationWorldToControl.rotate(positionDesiredFootHoldOnTerrainFeedBackInWorldFrame);
  positionDesiredFootHoldOnTerrainFeedBackInControlFrame.z() = 0.0; // project on terrain
  return positionDesiredFootHoldOnTerrainFeedBackInControlFrame;
}


// Evaluate the interpolation phase relative to a given leg
double FootPlacementStrategyFreePlane::getInterpolationPhase(const LegBase& leg)  {
  double swingPhase = 1;
  if (!leg.isSupportLeg()) {
    swingPhase = leg.getSwingPhase();
  }
  return mapTo01Range(swingPhase, leg.getStateLiftOff().getSwingPhase(), 1.0);
}


Position FootPlacementStrategyFreePlane::getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(const LegBase& leg) {
  const RotationQuaternion& orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();

  positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg.getId()] = getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame(leg);
  positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg.getId()] = getPositionDesiredFootHoldOnTerrainFeedBackInControlFrame(leg);

  Position positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame = positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg.getId()]
                                                                                     + positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg.getId()];

  /* TESTING */
  //--- add offset
//  Position offset = getOffsetDesiredFootOnTerrainToCorrectedFootOnTerrainInControlFrame(leg);
//  std::cout << "offset: " << offset << std::endl;
//  positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame -= offset;
  //---

  //--- save for debug
  //Position positionWorldToHipOnPlaneAlongNormalInWorldFrame = getPositionProjectedOnPlaneAlongSurfaceNormal(leg.getWorldToHipPositionInWorldFrame());

  Position positionWorldToHipVerticalOnPlaneInWorldFrame = leg.getPositionWorldToHipInWorldFrame();
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
