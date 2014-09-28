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
    gotFirstTouchDownOfFoot_(legs_->size()),
    heightMemory_(100),
    heightHorizontalPlaneAlgorithm_(0.0),
    planeParameters_(3)
  {
    for (auto leg: *legs_) {
      mostRecentPositionOfFoot_[leg->getId()].setZero();
      lastWorldToBasePositionInWorldFrameForFoot_[leg->getId()].setZero();
      lastWorldToBaseOrientationForFoot_[leg->getId()].setIdentity();
      gotFirstTouchDownOfFoot_[leg->getId()] = false;
    }

    for (int k=0; k<heightMemory_.size(); k++) {
      heightMemory_[k] = 0.0;
    }
    heightMemoryIndex_ = 0;

    for (int k=0; k<planeParameters_.size(); k++) {
    	planeParameters_[k] = 0.0;
    }
//
//    updateControlFrameOrigin();
//    updateControlFrameAttitude();

  } // constructor


  TerrainPerceptionFreePlane::~TerrainPerceptionFreePlane() {

  } // destructor


  bool TerrainPerceptionFreePlane::initialize(double dt) {
    for (auto leg: *legs_) {
      gotFirstTouchDownOfFoot_[leg->getId()] = false;
      updateLocalMeasuresOfLeg(*leg);
    } // for

    for (int k=0; k<heightMemory_.size(); k++) {
      heightMemory_[k] = 0.0;
    }
    heightMemoryIndex_ = 0;
    heightHorizontalPlaneAlgorithm_ = 0.0;

    updateControlFrameOrigin();
    updateControlFrameAttitude();

    return true;
  } // initialize


  void TerrainPerceptionFreePlane::updateControlFrameOrigin() {

    loco::Position positionWorldToMiddleOfFeetInWorldFrame;

    for (auto leg : *legs_) {
      positionWorldToMiddleOfFeetInWorldFrame += leg->getWorldToFootPositionInWorldFrame();
    }
    positionWorldToMiddleOfFeetInWorldFrame /= legs_->size();

    terrainModel_->getHeight(positionWorldToMiddleOfFeetInWorldFrame);
//    torso_->getMeasuredState().setPositionWorldToControlInWorldFrame(positionWorldToMiddleOfFeetInWorldFrame);


    Position positionWorldToBaseInWorldFrame = torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
    terrainModel_->getHeight(positionWorldToBaseInWorldFrame);
    torso_->getMeasuredState().setPositionWorldToControlInWorldFrame(positionWorldToBaseInWorldFrame);

  }


  void TerrainPerceptionFreePlane::updateControlFrameAttitude() {
    double terrainPitch, terrainRoll, controlFrameYaw;
    loco::Vector normalInWorldFrame;
    terrainModel_->getNormal(loco::Position::Zero(), normalInWorldFrame);

    //--- Get current heading direction
    const Position positionForeHipsMidPointInWorldFrame = (legs_->getLeftForeLeg()->getWorldToHipPositionInWorldFrame() + legs_->getRightForeLeg()->getWorldToHipPositionInWorldFrame())*0.5;
    const Position positionHindHipsMidPointInWorldFrame = (legs_->getLeftHindLeg()->getWorldToHipPositionInWorldFrame() + legs_->getRightHindLeg()->getWorldToHipPositionInWorldFrame())*0.5;
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

    //--- hack set control frame equal to heading frame
    EulerAnglesZyx orientationWorldToHeadingEulerZyx = EulerAnglesZyx(torso_->getMeasuredState().getOrientationWorldToBase()).getUnique();
    orientationWorldToHeadingEulerZyx.setPitch(0.0);
    orientationWorldToHeadingEulerZyx.setRoll(0.0);
    orientationWorldToControl = RotationQuaternion(orientationWorldToHeadingEulerZyx.getUnique());
    //---

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

    // Update terrain model properties (if necessary) based on current estimation
    if (gotNewTouchDown && allLegsGroundedAtLeastOnce) {
      updatePlaneEstimation();
    }

    computeEstimationNoise();

    terrainModel_->advance(dt);

    updateControlFrameOrigin();
    updateControlFrameAttitude();

    return true;
  } // advance


  void TerrainPerceptionFreePlane::computeEstimationNoise() {

    /*
     * 1. evaluate height as in horizontal plane algorithm
     * 2. subtract the average from it, so that one has an estimation of the noise coming from the robot estimator
     * 3. set the height to the terrain model, so that it can be added to the height it returns to other objects
     */

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
      // 1.
      heightHorizontalPlaneAlgorithm_ = gHeight / groundedLimbCount;

      // 2.
      heightMemory_[heightMemoryIndex_] = gHeight / groundedLimbCount;
      heightMemoryIndex_++;
      if (heightMemoryIndex_ == heightMemory_.size()) {
        heightMemoryIndex_ = 0;
      }
      double averageHeight = 0.0;

      for (int k=0; k<heightMemory_.size(); k++) {
        averageHeight += heightMemory_[k]/heightMemory_.size();
      }

      // 3.
      terrainModel_->setHeight(heightHorizontalPlaneAlgorithm_, torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame());
      terrainModel_->setHeightNoise(heightHorizontalPlaneAlgorithm_-averageHeight);
    } // if groundedLimbCount
    //---

  }


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

         for (int k=0; k<planeParameters_.size(); k++) {
        	 planeParameters_[k] = parameters[k];
         }

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
