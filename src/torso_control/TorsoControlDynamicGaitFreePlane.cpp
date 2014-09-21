/*
 * TorsoControlDynamicGaitFreePlane.cpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Dario Bellicoso
 */

#include "loco/torso_control/TorsoControlDynamicGaitFreePlane.hpp"
#include "loco/temp_helpers/math.hpp"

#include <exception>
namespace loco {

  TorsoControlDynamicGaitFreePlane::TorsoControlDynamicGaitFreePlane(LegGroup* legs, TorsoBase* torso,  loco::TerrainModelBase* terrain):
    TorsoControlDynamicGait(legs, torso, terrain),
    maxDesiredPitchRadians_(5.0*M_PI/180.0),
    desiredPitchSlope_(1.0),
    maxDesiredRollRadians_(5.0*M_PI/180.0),
    desiredRollSlope_(1.0),
    adaptToTerrain_(CompleteAdaption)
  {
    const double defaultHeight = 0.42;
    desiredTorsoCoMHeightAboveGroundInWorldFrameOffset_  = defaultHeight;

    firstOrderFilter_ = new loco::FirstOrderFilter();

  }


  TorsoControlDynamicGaitFreePlane::~TorsoControlDynamicGaitFreePlane() {
    delete firstOrderFilter_;
  }


  bool TorsoControlDynamicGaitFreePlane::initialize(double dt) {
    const Position foreHipPosition = legs_->getLeg(0)->getWorldToHipPositionInBaseFrame();
    const Position hindHipPosition = legs_->getLeg(2)->getWorldToHipPositionInBaseFrame();
    headingDistanceFromForeToHindInBaseFrame_ = foreHipPosition.x()-hindHipPosition.x();

    firstOrderFilter_->initialize(0.0, 1.0, 1.0);

    return true;
  }


  bool TorsoControlDynamicGaitFreePlane::advance(double dt) {

    comControl_.advance(dt);

    /*******************************************
     * Set desired CoM position in world frame *
     *******************************************/
    // evaluate desired CoM position in world frame
    Position desiredTorsoPositionInWorldFrame = comControl_.getDesiredWorldToCoMPositionInWorldFrame();

    // update desired CoM position height as a function of the estimated terrain height and the measured velocity in base frame
    loco::LinearVelocity measuredTorsoVelocityInBaseFrame = torso_->getMeasuredState().getBaseLinearVelocityInBaseFrame();
    measuredTorsoVelocityInBaseFrame.y() = 0;
    measuredTorsoVelocityInBaseFrame.z() = 0;

    double measuredTorsoVelocityInBaseFrameNorm = fabs( measuredTorsoVelocityInBaseFrame.norm() );
    double heightOffset = 0.1*( 1.0 - exp(-measuredTorsoVelocityInBaseFrameNorm/0.25) );

    //heightOffset = firstOrderFilter_->advance(dt, heightOffset);

    //std::cout << "vel: " << measuredTorsoVelocityInBaseFrameNorm << "filt. height offset: " << heightOffset << std::endl;

    terrain_->getHeight(desiredTorsoPositionInWorldFrame);
    desiredTorsoPositionInWorldFrame.z() += desiredTorsoCoMHeightAboveGroundInWorldFrameOffset_;
    //desiredTorsoPositionInWorldFrame.z() -= heightOffset;
    desiredTorsoPositionInWorldFrame += desiredPositionOffetInWorldFrame_;
    /***********************************************
     * End set desired CoM position in world frame *
     ***********************************************/

    /***************************
     * Set desired orientation *
     *
     * The desired orientation will be given by a composition of rotations:
     *    --> rotation from world to current heading (starting point for the composition).
     *    --> rotation from current heading to desired heading. This can be due to the body yaw induced by the foot holds.
     *    --> rotation from desired heading to desired base. This is an adaption to the terrain's estimated pitch and roll angles.
     *
     ***************************/
    // Get measured orientation
    const RotationQuaternion orientationWorldToHeading = torso_->getMeasuredState().getWorldToHeadingOrientation(); // --> current heading orientation
    RotationQuaternion orientationHeadingToDesiredHeading;                                                          // --> rotation due to desired twist
    RotationQuaternion orientationDesiredHeadingToDesiredBody;                                                      // --> rotation due to desired pitch/roll

    //--- Get desired heading direction
    const Position positionForeFeetMidPointInWorldFrame = (legs_->getLeftForeLeg()->getWorldToFootPositionInWorldFrame() + legs_->getRightForeLeg()->getWorldToFootPositionInWorldFrame())*0.5;
    const Position positionHindFeetMidPointInWorldFrame = (legs_->getLeftHindLeg()->getWorldToFootPositionInWorldFrame() + legs_->getRightHindLeg()->getWorldToFootPositionInWorldFrame())*0.5;
    Position positionWorldToDesiredForeFeetMidPointInWorldFrame = positionForeFeetMidPointInWorldFrame + comControl_.getPositionErrorVectorInWorldFrame();
    Position positionWorldToDesiredHindFeetMidPointInWorldFrame = positionHindFeetMidPointInWorldFrame + comControl_.getPositionErrorVectorInWorldFrame();
    Vector desiredHeadingDirectionInWorldFrame = Vector(positionWorldToDesiredForeFeetMidPointInWorldFrame-positionWorldToDesiredHindFeetMidPointInWorldFrame);
    desiredHeadingDirectionInWorldFrame.z() = 0.0;
    //---

    //--- Get current heading direction
    const Position positionForeHipsMidPointInWorldFrame = (legs_->getLeftForeLeg()->getWorldToHipPositionInWorldFrame() + legs_->getRightForeLeg()->getWorldToHipPositionInWorldFrame())*0.5;
    const Position positionHindHipsMidPointInWorldFrame = (legs_->getLeftHindLeg()->getWorldToHipPositionInWorldFrame() + legs_->getRightHindLeg()->getWorldToHipPositionInWorldFrame())*0.5;
    Vector currentHeadingDirectionInWorldFrame = Vector(positionForeHipsMidPointInWorldFrame-positionHindHipsMidPointInWorldFrame);
    currentHeadingDirectionInWorldFrame.z() = 0.0;
    //---

    try {
      orientationHeadingToDesiredHeading.setFromVectors(currentHeadingDirectionInWorldFrame.toImplementation(), desiredHeadingDirectionInWorldFrame.toImplementation());
    } catch (std::exception& e) {
      std::cout << e.what() << '\n';
      std::cout << "currentHeadingDirectionInWorldFrame: " << currentHeadingDirectionInWorldFrame <<std::endl;
      std::cout << "desiredHeadingDirectionInWorldFrame: " << desiredHeadingDirectionInWorldFrame <<std::endl;
      orientationHeadingToDesiredHeading.setIdentity();
    }

    //--- Get terrain estimated attitude and base desired attitude
    double desiredBasePitch, desiredBaseRoll;
    const loco::RotationQuaternion orientationWorldToDesiredHeading = orientationHeadingToDesiredHeading*orientationWorldToHeading;
    loco::Vector normalToPlaneInWorldFrame, normalToPlaneInDesiredHeadingFrame;

    terrain_->getNormal(loco::Position::Zero(), normalToPlaneInWorldFrame);
    normalToPlaneInDesiredHeadingFrame = orientationWorldToDesiredHeading.rotate(normalToPlaneInWorldFrame);

    getDesiredBasePitchFromTerrainPitch(atan2(normalToPlaneInDesiredHeadingFrame.x(), normalToPlaneInDesiredHeadingFrame.z()), desiredBasePitch);
    getDesiredBaseRollFromTerrainRoll(atan2(normalToPlaneInDesiredHeadingFrame.y(), normalToPlaneInDesiredHeadingFrame.z()), desiredBaseRoll);
    //---

    //--- Get rotation between normal to body in desired heading frame and normal to plane
    RotationQuaternion orientationDesiredHeadingToBasePitch = RotationQuaternion(AngleAxis(desiredBasePitch, 0.0, 1.0, 0.0));
    RotationQuaternion orientationDesiredHeadingToBaseRoll  = RotationQuaternion(AngleAxis(desiredBaseRoll, -1.0, 0.0, 0.0));
    //---

    //--- Compose rotations
    RotationQuaternion desOrientationWorldToBase = orientationDesiredHeadingToBaseRoll
                                                   * orientationDesiredHeadingToBasePitch
                                                   * orientationWorldToDesiredHeading;
    //---
    /*******************************
     * End set desired orientation *
     *******************************/

    // Set desired pose of base frame with respect to world frame
    torso_->getDesiredState().setWorldToBasePoseInWorldFrame(Pose(desiredTorsoPositionInWorldFrame, desOrientationWorldToBase));

    /* if a stance leg lost contact, lower it towards the normal to the terrain to re-gain contact */
    for (auto leg : *legs_) {
      if (leg->isInStanceMode()) {
        Position positionWorldToFootInWorldFrame =  leg->getWorldToFootPositionInWorldFrame();

        if (!leg->isGrounded()) {
          positionWorldToFootInWorldFrame -= 0.01*(loco::Position)normalToPlaneInWorldFrame;
        }

        const Position positionWorldToBaseInWorldFrame = torso_->getMeasuredState().getWorldToBasePositionInWorldFrame();
        const Position positionBaseToFootInWorldFrame = positionWorldToFootInWorldFrame - positionWorldToBaseInWorldFrame;
        const Position positionBaseToFootInBaseFrame = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame().rotate(positionBaseToFootInWorldFrame);
        leg->setDesiredJointPositions(leg->getJointPositionsFromBaseToFootPositionInBaseFrame(positionBaseToFootInBaseFrame));
      }
    }

    return true;
  }


  template <typename T> int TorsoControlDynamicGaitFreePlane::sgn(T val) {
      return (T(0) < val) - (val < T(0));
  }


  void TorsoControlDynamicGaitFreePlane::getDesiredBasePitchFromTerrainPitch(const double terrainPitch, double& desiredBasePitch) {
    if (adaptToTerrain_ == AdaptToTerrain::CompleteAdaption) {
      desiredBasePitch = terrainPitch;
    }
    else if (adaptToTerrain_ == AdaptToTerrain::SaturatedLinearAdaption) {
      if (fabs(terrainPitch) < maxDesiredPitchRadians_) {
        desiredBasePitch = terrainPitch;
      }
      else {
        desiredBasePitch = maxDesiredPitchRadians_*sgn(terrainPitch);
      }
    }
    else {
      /* This is redundant for now (see AdaptToTerrain::CompleteAdaption), but it should be kept here for safety if other enum cases are added
       * and if AdaptToTerrain::CompleteAdaptation changes in the future
       */
      desiredBasePitch = terrainPitch;
    }
  }


  void TorsoControlDynamicGaitFreePlane::getDesiredBaseRollFromTerrainRoll(const double terrainRoll, double& desiredBaseRoll) {
    if (adaptToTerrain_ == AdaptToTerrain::CompleteAdaption) {
        desiredBaseRoll = terrainRoll;
      }
      else if (adaptToTerrain_ == AdaptToTerrain::SaturatedLinearAdaption) {
        if (fabs(terrainRoll) < maxDesiredRollRadians_) {
          desiredBaseRoll = terrainRoll;
        }
        else {
          desiredBaseRoll = maxDesiredPitchRadians_*sgn(terrainRoll);
        }
      }
      else {
        /* This is redundant for now (see AdaptToTerrain::CompleteAdaption), but it should be kept here for safety if other enum cases are added
         * and if AdaptToTerrain::CompleteAdaptation changes in the future
         */
        desiredBaseRoll = terrainRoll;
      }
  }

} /* namespace loco */
