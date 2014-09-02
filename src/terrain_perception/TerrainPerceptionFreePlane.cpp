/*
 * TerrainPerceptionFreePlane.cpp
 *
 *  Created on: Aug 28, 2014
 *      Author: C. Dario Bellicoso
 */

#include "loco/terrain_perception/TerrainPerceptionFreePlane.hpp"

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
    numberOfLegs_(legs_->size()),
    mostRecentPositionOfFoot_(legs_->size()),
    lastWorldToBasePositionInWorldFrameForFoot_(legs_->size()),
    lastWorldToBaseOrientationForFoot_(4)
  {

    for (int k=0; k<numberOfLegs_; k++) {
      mostRecentPositionOfFoot_[k].setZero();
      lastWorldToBasePositionInWorldFrameForFoot_[k] = torso_->getMeasuredState().getWorldToBasePositionInWorldFrame();
      lastWorldToBaseOrientationForFoot_[k] = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();
    }

  } // constructor


  TerrainPerceptionFreePlane::~TerrainPerceptionFreePlane() {

  } // desctuctor


  bool TerrainPerceptionFreePlane::initialize(double dt) {
    int legID = 0;
    loco::Position footPosition;
    footPosition.setZero();

    // initialize foot positions
    for (auto leg: *legs_) {

      switch (estimatePlaneInFrame_) {
        case(EstimatePlaneInFrame::World): {
          mostRecentPositionOfFoot_[legID] = leg->getWorldToFootPositionInWorldFrame();
        } break;

        case(EstimatePlaneInFrame::Base): {
          mostRecentPositionOfFoot_[legID] = leg->getBaseToFootPositionInBaseFrame();
          lastWorldToBasePositionInWorldFrameForFoot_[legID] = torso_->getMeasuredState().getWorldToBasePositionInWorldFrame();
          lastWorldToBaseOrientationForFoot_[legID] = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();
        } break;

        default: {
          error: throw std::out_of_range("Index out of range ...");
        }
      } //switch

      legID++;
    } // for

    //updatePlaneEstimation();

    return true;

  } // initialize


  bool TerrainPerceptionFreePlane::advance(double dt) {
    int legID = 0;
    bool gotNewTouchDown = false;
    loco::Position footPosition;
    footPosition.setZero();

    for (auto leg: *legs_) {

      if ( leg->isGrounded() ) {
        gotNewTouchDown = true;

        switch (estimatePlaneInFrame_) {
          case(EstimatePlaneInFrame::World): {
            mostRecentPositionOfFoot_[legID] = leg->getWorldToFootPositionInWorldFrame();
          } break;

          case(EstimatePlaneInFrame::Base): {
            mostRecentPositionOfFoot_[legID] = leg->getBaseToFootPositionInBaseFrame();
            lastWorldToBasePositionInWorldFrameForFoot_[legID] = torso_->getMeasuredState().getWorldToBasePositionInWorldFrame();
            lastWorldToBaseOrientationForFoot_[legID] = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();
          } break;

          default: {
            error: throw std::out_of_range("Index out of range ...");
          }
        } //switch

      } // if touchdown
      legID++;
    }

    if (gotNewTouchDown) {
      updatePlaneEstimation();
    }

    return true;

  } // advance


  void TerrainPerceptionFreePlane::rotatePassiveFromBaseToWorldFrame(loco::Position& position) {
    RotationQuaternion rot;
    rot = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();
    position = rot.inverseRotate(position);
  }


  void TerrainPerceptionFreePlane::homogeneousTransformFromBaseToWorldFrame(loco::Position& position) {
    RotationQuaternion rot;
    loco::Position positionRotated;
    rot = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();

    positionRotated = rot.inverseRotate(position);
    position = torso_->getMeasuredState().getWorldToBasePositionInWorldFrame()
              + positionRotated;
  }


  void TerrainPerceptionFreePlane::homogeneousTransformFromBaseToWorldFrame(loco::Position& position, int footID) {
      loco::Position positionRotated;
      positionRotated = lastWorldToBaseOrientationForFoot_[footID].inverseRotate(position);
      position = lastWorldToBasePositionInWorldFrameForFoot_[footID]
                 + positionRotated;
  }


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
    loco::Vector normal;
    loco::Position position;

    linearRegressor.setZero();
    measuredFootHeights.setZero();
    parameters.setZero();
    normal.setZero();

    linearRegressor << -mostRecentPositionOfFoot_[0].x(), -mostRecentPositionOfFoot_[0].y(), 1,
                       -mostRecentPositionOfFoot_[1].x(), -mostRecentPositionOfFoot_[1].y(), 1,
                       -mostRecentPositionOfFoot_[2].x(), -mostRecentPositionOfFoot_[2].y(), 1,
                       -mostRecentPositionOfFoot_[3].x(), -mostRecentPositionOfFoot_[3].y(), 1;
    measuredFootHeights << mostRecentPositionOfFoot_[0].z(),
                           mostRecentPositionOfFoot_[1].z(),
                           mostRecentPositionOfFoot_[2].z(),
                           mostRecentPositionOfFoot_[3].z();

    kindr::linear_algebra::pseudoInverse(linearRegressor,linearRegressor);
    parameters = linearRegressor*measuredFootHeights;

    /* find a point on the plane. From z = d-ax-by, it is easy to find that p = [0 0 d]
     * is on the plane
     */
    position << 0.0, 0.0, parameters(2);

    /* from the assumption that the normal has always unit z-component,
     * its norm will always be greater than zero
     */
    normal << parameters(0), parameters(1), 1.0;
    normal = normal.normalize();

    if (estimatePlaneInFrame_ == EstimatePlaneInFrame::Base) {
      rotatePassiveFromBaseToWorldFrame((loco::Position&)normal);
      homogeneousTransformFromBaseToWorldFrame(position);
    }

    terrainModel_->setNormalandPositionInWorldFrame(normal, position);

  } // update plane estimation


  void TerrainPerceptionFreePlane::getMostRecentPositionOfFootInWorldFrame(loco::Position& footPositionInWorldFrame, int footID) {
    if (estimatePlaneInFrame_ == EstimatePlaneInFrame::Base) {
      /*
      loco::Position footPosition = mostRecentPositionOfFoot_[footID];
      homogeneousTransformFromBaseToWorldFrame(footPosition);
      footPositionInWorldFrame = footPosition;
      */

      loco::Position footPosition = mostRecentPositionOfFoot_[footID];
      homogeneousTransformFromBaseToWorldFrame(footPosition, footID);
      footPositionInWorldFrame = footPosition;
    }
    else {
      footPositionInWorldFrame = mostRecentPositionOfFoot_[footID];
    }
  } // get foot position


} /* namespace loco */
