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
    const double defaultHeight = 0.35;
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

    // Get measured orientation
        const RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl(); // --> current heading orientation
        RotationQuaternion orientationControlToDesiredHeading;                                                          // --> rotation due to desired twist

    /*******************************************
     * Set desired CoM position in world frame *
     *******************************************/
    // evaluate desired CoM position in control frame
        Position positionControlToHorizontalTargetBaseInControlFrame = orientationWorldToControl.rotate(comControl_.getDesiredWorldToCoMPositionInWorldFrame() - torso_->getMeasuredState().getPositionWorldToControlInWorldFrame());

        loco::Position positionWorldToMiddleOfFeetInWorldFrame;
        for (auto leg : *legs_) {
          positionWorldToMiddleOfFeetInWorldFrame += leg->getWorldToFootPositionInWorldFrame();
        }
        positionWorldToMiddleOfFeetInWorldFrame /= legs_->size();
        positionWorldToMiddleOfFeetInWorldFrame.z() = 0.0;


    const Position horizontalPositionErrorInWorldFrame = comControl_.getDesiredWorldToCoMPositionInWorldFrame() - positionWorldToMiddleOfFeetInWorldFrame;

    Position positionHorizontalControlToHorizontalBaseInWorldFrame = orientationWorldToControl.inverseRotate(torso_->getMeasuredState().getPositionControlToBaseInControlFrame())+ horizontalPositionErrorInWorldFrame;
    positionHorizontalControlToHorizontalBaseInWorldFrame.z() = 0.0;

    //Position positionControlToBaseInWorldFrame = positionHorizontalControlToHorizontalBaseInWorldFrame;
    //terrain_->getHeight(positionControlToBaseInWorldFrame);
    //double distanceVerticalDesiredBaseFromTerrain = 0.0;
    //positionControlToBaseInWorldFrame += distanceVerticalDesiredBaseFromTerrain*loco::Position::UnitZ();

    loco::Vector surfaceNormalInControlFrame;
    terrain_->getNormal(loco::Position::Zero(), surfaceNormalInControlFrame);
    orientationWorldToControl.rotate(surfaceNormalInControlFrame);

    loco::Vector axisWorldZInControlFrame = loco::Vector::UnitZ();
    orientationWorldToControl.rotate(axisWorldZInControlFrame);
    Position temp = orientationWorldToControl.rotate(positionHorizontalControlToHorizontalBaseInWorldFrame);

    Position temp2 = (Position)(desiredTorsoCoMHeightAboveGroundInWorldFrameOffset_*surfaceNormalInControlFrame)
           - temp;
    double scaleNumerator = Position(surfaceNormalInControlFrame).dot(temp2);
    double scaleDenominator = Position(surfaceNormalInControlFrame).dot(orientationWorldToControl.rotate(axisWorldZInControlFrame));

    double scale = scaleNumerator/scaleDenominator;

    Position positionControlToTargetBaseInControlFrame = orientationWorldToControl.rotate(positionHorizontalControlToHorizontalBaseInWorldFrame)
                                                         +Position(axisWorldZInControlFrame*scale);





//    double heightAtDesiredWorldToCoMPositionInWorldFrame;
//    terrain_->getHeight(comControl_.getDesiredWorldToCoMPositionInWorldFrame(), heightAtDesiredWorldToCoMPositionInWorldFrame);
//    // Correct the height to project
//    horizontalPositionErrorInWorldFrame.z() += heightAtDesiredWorldToCoMPositionInWorldFrame;


    /*
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
    desiredTorsoPositionInWorldFrame += desiredPositionOffsetInWorldFrame_;
    */



//    std::cout << "position com height in control frame: " << positionControlToTargetBaseInControlFrame
//              << " control frame height: " << torso_->getMeasuredState().getPositionWorldToControlInWorldFrame().z() << std::endl;

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
    //--- Get desired heading direction
    const Position positionForeFeetMidPointInWorldFrame = (legs_->getLeftForeLeg()->getWorldToFootPositionInWorldFrame() + legs_->getRightForeLeg()->getWorldToFootPositionInWorldFrame())*0.5;
    const Position positionHindFeetMidPointInWorldFrame = (legs_->getLeftHindLeg()->getWorldToFootPositionInWorldFrame() + legs_->getRightHindLeg()->getWorldToFootPositionInWorldFrame())*0.5;

//    Position positionControlToTargetBaseInControlFrame = positionControlToHorizontalTargetBaseInControlFrame
//                                                        + desiredTorsoCoMHeightAboveGroundInWorldFrameOffset_*loco::Position::UnitZ()
//                                                        + desiredPositionOffsetInWorldFrame_;

    Position positionWorldToDesiredForeFeetMidPointInWorldFrame = positionForeFeetMidPointInWorldFrame;// + positionControlToTargetBaseInControlFrame;
    Position positionWorldToDesiredHindFeetMidPointInWorldFrame = positionHindFeetMidPointInWorldFrame;// + positionControlToTargetBaseInControlFrame;

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
      orientationControlToDesiredHeading.setFromVectors(
          orientationWorldToControl.rotate(currentHeadingDirectionInWorldFrame.toImplementation()),
          orientationWorldToControl.rotate(desiredHeadingDirectionInWorldFrame.toImplementation())
          );
    } catch (std::exception& e) {
      std::cout << e.what() << '\n';
      std::cout << "currentHeadingDirectionInWorldFrame: " << currentHeadingDirectionInWorldFrame <<std::endl;
      std::cout << "desiredHeadingDirectionInWorldFrame: " << desiredHeadingDirectionInWorldFrame <<std::endl;
      orientationControlToDesiredHeading.setIdentity();
    }

    //--- Compose rotations
    //desiredOrientationOffset_ = RotationQuaternion(AngleAxis(4.0*M_PI/180.0,0.0,1.0,0.0));
    RotationQuaternion orientationControlToDesiredBase = desiredOrientationOffset_*orientationControlToDesiredHeading;
    //std::cout << "desired orient: " << EulerAnglesZyx(orientationControlToDesiredBase).getUnique() << std::endl;
    //---

    /*******************************
     * End set desired orientation *
     *******************************/

    // Set desired pose of base frame with respect to world frame
    //torso_->getDesiredState().setWorldToBasePoseInWorldFrame(Pose(desiredTorsoPositionInWorldFrame, desOrientationWorldToBase));
    torso_->getDesiredState().setPositionControlToBaseInControlFrame(positionControlToTargetBaseInControlFrame);
    //torso_->getDesiredState().setPositionControlToBaseInControlFrame(Position(0.0,0.0,0.4));
    torso_->getDesiredState().setOrientationControlToBase(orientationControlToDesiredBase);
    //torso_->getDesiredState().setOrientationControlToBase(desiredOrientationOffset_);


//    std::cout << "************" << std::endl;
//    std::cout << "des lin vel in control frame: " << torso_->getDesiredState().getLinearVelocityBaseInControlFrame() << std::endl;
//    std::cout << "control frame pos: " << torso_->getMeasuredState().getPositionWorldToControlInWorldFrame()
//              << " orientation: " << EulerAnglesZyx(orientationWorldToControl).getUnique() << std::endl;

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
