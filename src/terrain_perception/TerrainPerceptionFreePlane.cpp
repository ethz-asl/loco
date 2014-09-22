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
    numberOfLegs_(legs_->size()),
    mostRecentPositionOfFoot_(legs_->size()),
    lastWorldToBasePositionInWorldFrameForFoot_(legs_->size()),
    lastWorldToBaseOrientationForFoot_(legs_->size()),
    gotFirstTouchDownOfFoot_(legs_->size())
  {
    for (auto leg: *legs_) {
      mostRecentPositionOfFoot_[leg->getId()].setZero();
      lastWorldToBasePositionInWorldFrameForFoot_[leg->getId()].setZero();
      lastWorldToBaseOrientationForFoot_[leg->getId()].setIdentity();
      gotFirstTouchDownOfFoot_[leg->getId()] = false;
    }
  } // constructor


  TerrainPerceptionFreePlane::~TerrainPerceptionFreePlane() {

  } // destructor


  bool TerrainPerceptionFreePlane::initialize(double dt) {
    for (auto leg: *legs_) {
      gotFirstTouchDownOfFoot_[leg->getId()] = false;
      updateLocalMeasuresOfLeg(*leg);
    } // for
    return true;
  } // initialize


  bool TerrainPerceptionFreePlane::advance(double dt) {
    bool gotNewTouchDown = false;
    bool allLegsGroundedAtLeastOnce = true;
    int legID = 0;

    for (auto leg: *legs_) {
      legID = leg->getId();
      if ( leg->getStateTouchDown()->isNow() ) {
        gotNewTouchDown = true;
        gotFirstTouchDownOfFoot_[legID] = true;
        updateLocalMeasuresOfLeg(*leg);
      } // if touchdown

      allLegsGroundedAtLeastOnce *= gotFirstTouchDownOfFoot_[legID];
    } // for

    if (gotNewTouchDown && allLegsGroundedAtLeastOnce) { updatePlaneEstimation(); }


    //--- UPDATE HEIGHT AS IN HORIZONTAL PLANE PERCEPTION
    int groundedLimbCount = 0;
      double gHeight = 0.0;


      for (auto leg : *legs_) {
        if (leg->isAndShouldBeGrounded()){
          groundedLimbCount++;
          gHeight += leg->getWorldToFootPositionInWorldFrame().z();
        }
      }

      if (groundedLimbCount > 0) {
        terrainModel_->setHeight(gHeight / groundedLimbCount);
      }
    //---

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
        mostRecentPositionOfFoot_[legID] = leg.getWorldToFootPositionInWorldFrame();
      } break;

      case(EstimatePlaneInFrame::Base): {
        mostRecentPositionOfFoot_[legID] = leg.getBaseToFootPositionInBaseFrame();
        lastWorldToBasePositionInWorldFrameForFoot_[legID] = torso_->getMeasuredState().getWorldToBasePositionInWorldFrame();
        lastWorldToBaseOrientationForFoot_[legID] = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();
      } break;

      default: {
        error: throw std::out_of_range("Index out of range ...");
      }
    } // switch
  } // update local measures


  void TerrainPerceptionFreePlane::rotatePassiveFromBaseToWorldFrame(loco::Position& position) {
    RotationQuaternion rot;
    rot = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();
    position = rot.inverseRotate(position);
  } // passive rotation helper


  void TerrainPerceptionFreePlane::homogeneousTransformFromBaseToWorldFrame(loco::Position& position) {
    RotationQuaternion rot;
    loco::Position positionRotated;
    rot = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();

    positionRotated = rot.inverseRotate(position);
    position = torso_->getMeasuredState().getWorldToBasePositionInWorldFrame()
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
    loco::Vector normal;
    loco::Position position;

    linearRegressor.setZero();
    measuredFootHeights.setZero();
    parameters.setZero();
    normal.setZero();

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

         /* Find a point on the plane. From z = d-ax-by, it is easy to find that p = [0 0 d]
          * is on the plane
          */
         position << 0.0, 0.0, parameters(2);

         /* From the assumption that the normal has always unit z-component,
          * its norm will always be greater than zero
          */
         normal << parameters(0), parameters(1), 1.0;
         normal = normal.normalize();

         /* Update free plane model */
         terrainModel_->setNormalandPositionInWorldFrame(normal, position);

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
