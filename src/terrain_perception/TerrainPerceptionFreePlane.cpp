/*****************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, C. Dario Bellicoso, Christian Gehring, Péter Fankhauser, Stelian Coros
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
/*
 * TerrainPerceptionFreePlane.cpp
 *
 *  Created on: Aug 28, 2014
 *      Author: C. Dario Bellicoso
 */

#include "loco/terrain_perception/TerrainPerceptionFreePlane.hpp"


#define TERRAINPERCEPTION_DEBUG   0 /* change to 1 to print to print each leg's status to std::cout at a touchdown event */

namespace loco {

  TerrainPerceptionFreePlane::TerrainPerceptionFreePlane(TerrainModelFreePlane* terrainModel,
                                                         LegGroup* legs,
                                                         TorsoStarlETH* torso,
                                                         TerrainPerceptionFreePlane::EstimatePlaneInFrame estimatePlaneInFrame):
    TerrainPerceptionBase(),
    terrainModel_(terrainModel),
    legs_(legs),
    torso_(torso),
    estimatePlaneInFrame_(estimatePlaneInFrame),
    mostRecentPositionOfFoot_(legs_->size()),
    lastWorldToBasePositionInWorldFrameForFoot_(legs_->size()),
    lastWorldToBaseOrientationForFoot_(legs_->size()),
    gotFirstTouchDownOfFoot_(legs_->size()),
    planeParameters_(3),
    filterNormalTimeConstant_(0.05),
    filterPositionTimeConstant_(0.05),
    filterNormalGain_(1.0),
    filterPositionGain_(1.0)
  {
    for (auto leg: *legs_) {
      mostRecentPositionOfFoot_[leg->getId()].setZero();
      lastWorldToBasePositionInWorldFrameForFoot_[leg->getId()].setZero();
      lastWorldToBaseOrientationForFoot_[leg->getId()].setIdentity();
      gotFirstTouchDownOfFoot_[leg->getId()] = false;
    }

    for (int k=0; k<planeParameters_.size(); k++) {
    	planeParameters_[k] = 0.0;
    }

    filterNormalX_ = robotUtils::FirstOrderFilter();
    filterNormalY_ = robotUtils::FirstOrderFilter();
    filterNormalZ_ = robotUtils::FirstOrderFilter();
    filterPositionX_ = robotUtils::FirstOrderFilter();
    filterPositionY_ = robotUtils::FirstOrderFilter();
    filterPositionZ_ = robotUtils::FirstOrderFilter();
  } // constructor


  TerrainPerceptionFreePlane::~TerrainPerceptionFreePlane() {

  } // destructor


  bool TerrainPerceptionFreePlane::initialize(double dt) {
    for (auto leg: *legs_) {
      gotFirstTouchDownOfFoot_[leg->getId()] = false;
      updateLocalMeasuresOfLeg(*leg);
    }

    //--- Initialize normal and position vectors
    normalInWorldFrameFilterOutput_ = loco::Vector::UnitZ();
    positionInWorldFrameFilterOutput_.setZero();
    normalInWorldFrameFilterInput_ = normalInWorldFrameFilterOutput_;
    positionInWorldFrameFilterInput_ = positionInWorldFrameFilterOutput_;

    filterNormalTimeConstant_ = 0.05;
    filterPositionTimeConstant_ = 0.05;
    filterNormalGain_ = 1.0;
    filterPositionGain_ = 1.0;

    filterNormalX_.initialize(normalInWorldFrameFilterOutput_.x(), filterNormalTimeConstant_, filterNormalGain_);
    filterNormalY_.initialize(normalInWorldFrameFilterOutput_.y(), filterNormalTimeConstant_, filterNormalGain_);
    filterNormalZ_.initialize(normalInWorldFrameFilterOutput_.z(), filterNormalTimeConstant_, filterNormalGain_);

    filterPositionX_.initialize(positionInWorldFrameFilterOutput_.x(), filterPositionTimeConstant_, filterPositionGain_);
    filterPositionY_.initialize(positionInWorldFrameFilterOutput_.y(), filterPositionTimeConstant_, filterPositionGain_);
    filterPositionZ_.initialize(positionInWorldFrameFilterOutput_.z(), filterPositionTimeConstant_, filterPositionGain_);

    updateControlFrameOrigin();
    updateControlFrameAttitude();

    return true;
  } // initialize


  void TerrainPerceptionFreePlane::updateControlFrameOrigin() {
  /*********************************************************
    * METHOD I - NEW
    *********************************************************/
//    //--- Position of the control frame is equal to the position of the middle of the feet
//    //    projected on the terrain along the vertical axis of the world frame.
//    loco::Position positionWorldToMiddleOfFeetInWorldFrame;
//    for (auto leg : *legs_) {
//      positionWorldToMiddleOfFeetInWorldFrame += leg->getWorldToFootPositionInWorldFrame();
//    }
//    positionWorldToMiddleOfFeetInWorldFrame /= legs_->size();
//
//    terrainModel_->getHeight(positionWorldToMiddleOfFeetInWorldFrame);
//    torso_->getMeasuredState().setPositionWorldToControlInWorldFrame(positionWorldToMiddleOfFeetInWorldFrame);
//    //---
//
//    //--- Position of the control frame is equal to the position of the base frame
//    //    projected on the terrain along the vertical axis of the world frame.
//    const Position& positionWorldToBaseInWorldFrame = torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
//    Position positionWorldToControlInWorldFrame = positionWorldToBaseInWorldFrame;
//    terrainModel_->getHeight(positionWorldToControlInWorldFrame);
//    torso_->getMeasuredState().setPositionWorldToControlInWorldFrame(positionWorldToControlInWorldFrame);
//    //---


    /*********************************************************
     * METHOD II - Old
     *********************************************************/
    //--- Position of the control frame is equal to the position of the world frame.
    torso_->getMeasuredState().setPositionWorldToControlInWorldFrame(Position::Zero());
    //---
  }


  void TerrainPerceptionFreePlane::updateControlFrameAttitude() {
    /*********************************************************
      * METHOD I - NEW
      *********************************************************/
    double terrainPitch, terrainRoll, controlFrameYaw;
    loco::Vector normalInWorldFrame;
    terrainModel_->getNormal(loco::Position::Zero(), normalInWorldFrame);

    //--- Get current heading direction
    const Position positionForeHipsMidPointInWorldFrame = (legs_->getLeftForeLeg()->getPositionWorldToHipInWorldFrame() + legs_->getRightForeLeg()->getPositionWorldToHipInWorldFrame())*0.5;
    const Position positionHindHipsMidPointInWorldFrame = (legs_->getLeftHindLeg()->getPositionWorldToHipInWorldFrame() + legs_->getRightHindLeg()->getPositionWorldToHipInWorldFrame())*0.5;
    Vector currentHeadingDirectionInWorldFrame = Vector(positionForeHipsMidPointInWorldFrame-positionHindHipsMidPointInWorldFrame);
    currentHeadingDirectionInWorldFrame.z() = 0.0;

    RotationQuaternion orientationWorldToControlHeading;
    Eigen::Vector3d axisX = Eigen::Vector3d::UnitX();
    orientationWorldToControlHeading.setFromVectors(axisX, currentHeadingDirectionInWorldFrame.toImplementation());
    //---

    loco::Vector normalInHeadingControlFrame = orientationWorldToControlHeading.rotate(normalInWorldFrame);
    terrainPitch = atan2(normalInHeadingControlFrame.x(), normalInHeadingControlFrame.z());
    terrainRoll = atan2(normalInHeadingControlFrame.y(), normalInHeadingControlFrame.z());

    RotationQuaternion orientationWorldToControl = RotationQuaternion(AngleAxis(terrainRoll, -1.0, 0.0, 0.0))*RotationQuaternion(AngleAxis(terrainPitch, 0.0, 1.0, 0.0))*orientationWorldToControlHeading;


    /*********************************************************
     * METHOD II
     *********************************************************/
//    //--- hack set control frame equal to heading frame
//    RotationQuaternion orientationWorldToControl;
//    EulerAnglesZyx orientationWorldToHeadingEulerZyx = EulerAnglesZyx(torso_->getMeasuredState().getOrientationWorldToBase()).getUnique();
//    orientationWorldToHeadingEulerZyx.setPitch(0.0);
//    orientationWorldToHeadingEulerZyx.setRoll(0.0);
//    orientationWorldToControl = RotationQuaternion(orientationWorldToHeadingEulerZyx.getUnique());
//    //---


    torso_->getMeasuredState().setOrientationWorldToControl(orientationWorldToControl);
    torso_->getMeasuredState().setPositionControlToBaseInControlFrame(orientationWorldToControl.rotate(torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame() - torso_->getMeasuredState().getPositionWorldToControlInWorldFrame()));

    RotationQuaternion orientationWorldToBase = torso_->getMeasuredState().getOrientationWorldToBase();
    torso_->getMeasuredState().setOrientationControlToBase(orientationWorldToBase*orientationWorldToControl.inverted());

  }


  bool TerrainPerceptionFreePlane::advance(double dt) {
    bool gotNewTouchDown = false;
    bool allLegsGroundedAtLeastOnce = true;
    int legID = 0;

    // Update foot measurements
    for (auto leg: *legs_) {
      legID = leg->getId();
      //if ( leg->getStateTouchDown()->isNow() ) {
      if (leg->isAndShouldBeGrounded()) {
      //if (leg->isGrounded()) {
        gotNewTouchDown = true;
        gotFirstTouchDownOfFoot_[legID] = true;
        updateLocalMeasuresOfLeg(*leg);
      } // if touchdown

      allLegsGroundedAtLeastOnce *= gotFirstTouchDownOfFoot_[legID];
    } // for


    /* Sequence:
     * 1. update (if needed) normal and position
     * 2. filter
     * 3. set to terrain model
     * 4. update control frame
     */
    // 1. Update terrain model properties (if necessary) based on current estimation
    if (gotNewTouchDown && allLegsGroundedAtLeastOnce) {
      updatePlaneEstimation();
    }

    // 2. filter
    normalInWorldFrameFilterOutput_.x() = filterNormalX_.advance(dt, normalInWorldFrameFilterInput_.x());
    normalInWorldFrameFilterOutput_.y() = filterNormalY_.advance(dt, normalInWorldFrameFilterInput_.y());
    normalInWorldFrameFilterOutput_.z() = filterNormalZ_.advance(dt, normalInWorldFrameFilterInput_.z());

    positionInWorldFrameFilterOutput_.x() = filterPositionX_.advance(dt, positionInWorldFrameFilterInput_.x());
    positionInWorldFrameFilterOutput_.y() = filterPositionY_.advance(dt, positionInWorldFrameFilterInput_.y());
    positionInWorldFrameFilterOutput_.z() = filterPositionZ_.advance(dt, positionInWorldFrameFilterInput_.z());

    // 3.
    terrainModel_->setNormalandPositionInWorldFrame(normalInWorldFrameFilterOutput_, positionInWorldFrameFilterOutput_);

    // 4.
    updateControlFrameOrigin();
    updateControlFrameAttitude();

    return true;
  } // advance


  void TerrainPerceptionFreePlane::updateLocalMeasuresOfLeg(const loco::LegBase& leg) {
    int legID = leg.getId();

    #if TERRAINPERCEPTION_DEBUG
    std::cout << "updating on leg: " << legID << std::endl;
    std::cout << "leg n.: " << legID << " state:\n" << *leg << std::endl;
    #endif

    switch (estimatePlaneInFrame_) {

      case(EstimatePlaneInFrame::World): {
        mostRecentPositionOfFoot_[legID] = leg.getPositionWorldToFootInWorldFrame();
      } break;

      case(EstimatePlaneInFrame::Base): {
        mostRecentPositionOfFoot_[legID] = leg.getPositionBaseToFootInBaseFrame();
        lastWorldToBasePositionInWorldFrameForFoot_[legID] = torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
        lastWorldToBaseOrientationForFoot_[legID] = torso_->getMeasuredState().getOrientationWorldToBase();
      } break;

      default: {
        error: throw std::out_of_range("Index out of range ...");
      } break;

    } // switch
  } // update local measures


  void TerrainPerceptionFreePlane::rotatePassiveFromBaseToWorldFrame(loco::Position& position) {
    RotationQuaternion rot;
    rot = torso_->getMeasuredState().getOrientationWorldToBase();
    position = rot.inverseRotate(position);
  } // passive rotation helper


  void TerrainPerceptionFreePlane::homogeneousTransformFromBaseToWorldFrame(loco::Position& position) {
    RotationQuaternion rot;
    loco::Position positionRotated;
    rot = torso_->getMeasuredState().getOrientationWorldToBase();

    positionRotated = rot.inverseRotate(position);
    position = torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame()
              + positionRotated;
  } // homogeneous transform helper


  void TerrainPerceptionFreePlane::homogeneousTransformFromBaseToWorldFrame(loco::Position& position, int footID) {
      loco::Position positionRotated;
      positionRotated = lastWorldToBaseOrientationForFoot_[footID].inverseRotate(position);
      position = lastWorldToBasePositionInWorldFrameForFoot_[footID]
                 + positionRotated;
  } // foot homogeneous transform helper


  void TerrainPerceptionFreePlane::updatePlaneEstimation() {
    /* estimate the plane which best fits the most recent contact points of each foot in world frame
     * using least squares (pseudo inversion of the regressor matrix H)
     *
     * parameters       -> [a b d]^T
     * plane equation   -> z = d-ax-by
     * normal to plane  -> n = [a b 1]^T
     *
     * */
    Eigen::MatrixXd linearRegressor(4,3);
    Eigen::Vector4d measuredFootHeights;
    Eigen::Vector3d parameters;

    linearRegressor.setZero();
    measuredFootHeights.setZero();
    parameters.setZero();

    if (estimatePlaneInFrame_ == EstimatePlaneInFrame::Base) {
      std::vector<loco::Position> mostRecenPositionOfFootInWorldFrame(legs_->size());

      for (int k=0; k<legs_->size(); k++) {
        mostRecenPositionOfFootInWorldFrame[k] = mostRecentPositionOfFoot_[k];
        homogeneousTransformFromBaseToWorldFrame(mostRecenPositionOfFootInWorldFrame[k],k);
      }

      linearRegressor << -mostRecenPositionOfFootInWorldFrame[0].x(), -mostRecenPositionOfFootInWorldFrame[0].y(), 1,
                         -mostRecenPositionOfFootInWorldFrame[1].x(), -mostRecenPositionOfFootInWorldFrame[1].y(), 1,
                         -mostRecenPositionOfFootInWorldFrame[2].x(), -mostRecenPositionOfFootInWorldFrame[2].y(), 1,
                         -mostRecenPositionOfFootInWorldFrame[3].x(), -mostRecenPositionOfFootInWorldFrame[3].y(), 1;
      measuredFootHeights << mostRecenPositionOfFootInWorldFrame[0].z(),
                             mostRecenPositionOfFootInWorldFrame[1].z(),
                             mostRecenPositionOfFootInWorldFrame[2].z(),
                             mostRecenPositionOfFootInWorldFrame[3].z();
    }
    else {
      linearRegressor << -mostRecentPositionOfFoot_[0].x(), -mostRecentPositionOfFoot_[0].y(), 1,
                         -mostRecentPositionOfFoot_[1].x(), -mostRecentPositionOfFoot_[1].y(), 1,
                         -mostRecentPositionOfFoot_[2].x(), -mostRecentPositionOfFoot_[2].y(), 1,
                         -mostRecentPositionOfFoot_[3].x(), -mostRecentPositionOfFoot_[3].y(), 1;
      measuredFootHeights << mostRecentPositionOfFoot_[0].z(),
                             mostRecentPositionOfFoot_[1].z(),
                             mostRecentPositionOfFoot_[2].z(),
                             mostRecentPositionOfFoot_[3].z();
    }

    /* Check if the measurements are linearly dependent */
    Eigen::FullPivLU<Eigen::MatrixXd> piv_regressor(linearRegressor);
    if (piv_regressor.rank() < parameters.size() ) {
      std::cout << "*******WARNING: rank-deficient regressor. Skipping terrain update.*******" << std::endl;
    }
    else {
       if (kindr::linear_algebra::pseudoInverse(linearRegressor,linearRegressor)) {
         /* solve least squares problem */
         parameters = linearRegressor*measuredFootHeights;

         for (int k=0; k<planeParameters_.size(); k++) {
        	 planeParameters_[k] = parameters[k];
         }

         /* Find a point on the plane. From z = d-ax-by, it is easy to find that p = [0 0 d]
          * is on the plane
          */
         positionInWorldFrameFilterInput_ << 0.0, 0.0, parameters(2);

         /* From the assumption that the normal has always unit z-component,
          * its norm will always be greater than zero
          */
         normalInWorldFrameFilterInput_ << parameters(0), parameters(1), 1.0;
         normalInWorldFrameFilterInput_ = normalInWorldFrameFilterInput_.normalize();

       }
       else {
         std::cout << "*******WARNING: pseudoinversion returned error. Skipping terrain update.*******" << std::endl;
       }
    }

  } // update plane estimation


  void TerrainPerceptionFreePlane::getMostRecentPositionOfFootInWorldFrame(loco::Position& footPositionInWorldFrame, int footID) {
    if (estimatePlaneInFrame_ == EstimatePlaneInFrame::Base) {
      loco::Position footPosition = mostRecentPositionOfFoot_[footID];
      homogeneousTransformFromBaseToWorldFrame(footPosition, footID);
      footPositionInWorldFrame = footPosition;
    }
    else {
      footPositionInWorldFrame = mostRecentPositionOfFoot_[footID];
    }
  } // get foot position


} /* namespace loco */
