/*
 * TerrainPerceptionFreePlane.cpp
 *
 *  Created on: Aug 28, 2014
 *      Author: C. Dario Bellicoso
 */

#include "loco/terrain_perception/TerrainPerceptionFreePlane.hpp"

namespace loco {


  TerrainPerceptionFreePlane::TerrainPerceptionFreePlane(TerrainModelFreePlane* terrainModel, LegGroup* legs, TorsoStarlETH* torso, loco::EstimatePlaneInFrame estimatePlaneInFrame):
    TerrainPerceptionBase(),
    terrainModel_(terrainModel),
    legs_(legs),
    torso_(torso),
    estimatePlaneInFrame_(estimatePlaneInFrame),
    numberOfLegs_(4)
  {

    for (int k=0; k<numberOfLegs_; k++) {
      mostRecentPositionOfFoot_[k].setZero();
    }

  } // constructor


  TerrainPerceptionFreePlane::~TerrainPerceptionFreePlane() {

  } // desctuctor


  bool TerrainPerceptionFreePlane::initialize(double dt) {
    int legID = 0;

    // initialize foot positions
    for (auto leg: *legs_) {

      switch (estimatePlaneInFrame_) {
        case(EstimatePlaneInFrame::World): {
          mostRecentPositionOfFoot_[legID] = leg->getWorldToFootPositionInWorldFrame();
        } break;

        case(EstimatePlaneInFrame::Base): {
          mostRecentPositionOfFoot_[legID] = leg->getBaseToFootPositionInBaseFrame();
        } break;
      } //switch

      legID++;
    } // for

    return true;

  } // initialize


  bool TerrainPerceptionFreePlane::advance(double dt) {

    // update each foot position in base frame if they are grounded and were in swing mode.
    // the update will be done only on first contact of each stance phase.
    int legID = 0;
    bool gotNewTouchDown = false;

    for (auto leg: *legs_) {
      if ( leg->getStateTouchDown()->isNow() ) {

        gotNewTouchDown = true;

        switch (estimatePlaneInFrame_) {
          case(EstimatePlaneInFrame::World): {
            mostRecentPositionOfFoot_[legID] = leg->getWorldToFootPositionInWorldFrame();
          } break;

          case(EstimatePlaneInFrame::Base): {
            mostRecentPositionOfFoot_[legID] = leg->getBaseToFootPositionInBaseFrame();
          } break;
        } //switch

      }
      legID++;
    }

    if (gotNewTouchDown) {
      updatePlaneEstimation();
    }

    return true;

  } // advance


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
    normal << parameters(0), parameters(1), 1.0;

    /* from the assumption that the normal has always unit z-component,
     * its norm will always be greater than zero
     */
    normal = normal.normalize();
    parameters(2) /= normal.norm();

    /*
    if (estimatePlaneInFrame_ == EstimatePlaneInFrame::Base) {
      loco::Vector nearestPointToOrigin;
      //nearestPointToOrigin = nearestPointToOrigin/normal.norm()*(double)parameters(2); // in base frame
      double scale = parameters(2)/normal.norm();
      nearestPointToOrigin = scale*normal;

      RotationQuaternion rot;
      rot = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();

      normal = rot.rotate(normal);

      nearestPointToOrigin = (loco::Vector)torso_->getMeasuredState().getWorldToBasePositionInWorldFrame()
                             + (loco::Vector)rot.rotate(nearestPointToOrigin);

      parameters(2) = normal.dot(nearestPointToOrigin)*normal.norm();
    }
    */

    terrainModel_->setNormalAndConstantTerm(normal, (double)parameters(2));

  } // update plane estimation

} /* namespace loco */
