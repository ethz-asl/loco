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
    desiredTorsoCoMHeightAboveGroundInControlFrameOffset_  = defaultHeight;

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

    /*******************************************
     * Set desired CoM position in world frame *
     *******************************************/
    // evaluate desired CoM position in control frame

    // this is the center of the feet projected on the x-y plane of the world frame
    loco::Position positionWorldToMiddleOfFeetInWorldFrame;
    for (auto leg : *legs_) {
      positionWorldToMiddleOfFeetInWorldFrame += leg->getWorldToFootPositionInWorldFrame();
    }
    positionWorldToMiddleOfFeetInWorldFrame /= legs_->size();
    positionWorldToMiddleOfFeetInWorldFrame.z() = 0.0;

    // this is the error vector between the desired and measured location of the base projected on the x-y plane of the world frame and expressed in the world frame
    const Position horizontalPositionErrorInWorldFrame = comControl_.getDesiredWorldToCoMPositionInWorldFrame() - positionWorldToMiddleOfFeetInWorldFrame;

    // this is the desired location of the base location relative to the origin of the control frame projected on the x-y plane of the world frame and expressed in the world frame
//    Position positionHorizontalControlToHorizontalBaseInWorldFrame = orientationWorldToControl.inverseRotate(torso_->getMeasuredState().getPositionControlToBaseInControlFrame())+ horizontalPositionErrorInWorldFrame;
    Position positionHorizontalControlToHorizontalBaseInWorldFrame = comControl_.getDesiredWorldToCoMPositionInWorldFrame() - torso_->getMeasuredState().getPositionWorldToControlInWorldFrame();
    positionHorizontalControlToHorizontalBaseInWorldFrame.z() = 0.0;

    /*
     * The desired base position has to lie on the line:
     *    W_r_C_B* = W_r_C_Bh* + lambda*W_e_z^W
     * and on the plane parallel to the surface (with normal W_n) with the desired height above ground (h*):
     *    C_n * (C_r_C_B* - C_n * h*) = 0
     *
     *  The intersection point is given by:
     *                             C_n * ( C_n * h* - C_r_C_Bh*)
     *    C_r_C_B* = C_r_C_Bh*  +  ----------------------------- * C_e_z^W
     *                                     C_n * C_e_z^W
     *
     *   We compute it in control frame instead of world frame.
     *
     */

    // this is the suface normal
    loco::Vector surfaceNormalInWorldFrame;
    terrain_->getNormal(loco::Position::Zero(), surfaceNormalInWorldFrame);
    loco::Vector surfaceNormalInControlFrame = orientationWorldToControl.rotate(surfaceNormalInWorldFrame);

    const loco::Vector verticalAxisOfWorldFrameInWorldFrame = loco::Vector::UnitZ();
    const loco::Vector verticalAxisOfWorldFrameInControlFrame = orientationWorldToControl.rotate(verticalAxisOfWorldFrameInWorldFrame);


    const Position positionHorizontalControlToHorizontalBaseInControlFrame = orientationWorldToControl.rotate(positionHorizontalControlToHorizontalBaseInWorldFrame);

    Position temp2 = (desiredTorsoCoMHeightAboveGroundInControlFrameOffset_*Position(surfaceNormalInControlFrame))
           - positionHorizontalControlToHorizontalBaseInControlFrame;
    double scaleNumerator = Position(surfaceNormalInControlFrame).dot(temp2);
    double scaleDenominator = Position(surfaceNormalInControlFrame).dot(verticalAxisOfWorldFrameInControlFrame);

    double scale = scaleNumerator/scaleDenominator;

    // correct
    Position positionControlToTargetBaseInControlFrame = orientationWorldToControl.rotate(positionHorizontalControlToHorizontalBaseInWorldFrame)
                                                         + Position(verticalAxisOfWorldFrameInControlFrame*scale);
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
    //--- Get desired heading direction with respect to the current feet
    const Position positionForeFeetMidPointInWorldFrame = (legs_->getLeftForeLeg()->getWorldToFootPositionInWorldFrame() + legs_->getRightForeLeg()->getWorldToFootPositionInWorldFrame())*0.5;
    const Position positionHindFeetMidPointInWorldFrame = (legs_->getLeftHindLeg()->getWorldToFootPositionInWorldFrame() + legs_->getRightHindLeg()->getWorldToFootPositionInWorldFrame())*0.5;

//    Position positionControlToTargetBaseInControlFrame = positionControlToHorizontalTargetBaseInControlFrame
//                                                        + desiredTorsoCoMHeightAboveGroundInWorldFrameOffset_*loco::Position::UnitZ()
//                                                        + desiredPositionOffsetInWorldFrame_;

    Position positionWorldToDesiredForeFeetMidPointInWorldFrame = positionForeFeetMidPointInWorldFrame+horizontalPositionErrorInWorldFrame;// + positionControlToTargetBaseInControlFrame;
    Position positionWorldToDesiredHindFeetMidPointInWorldFrame = positionHindFeetMidPointInWorldFrame+horizontalPositionErrorInWorldFrame;// + positionControlToTargetBaseInControlFrame;

    Vector desiredHeadingDirectionInWorldFrame = Vector(positionWorldToDesiredForeFeetMidPointInWorldFrame-positionWorldToDesiredHindFeetMidPointInWorldFrame);
    desiredHeadingDirectionInWorldFrame.z() = 0.0;
    //---

    //--- Get current heading direction defined by the mid hip points
    const Position positionForeHipsMidPointInWorldFrame = (legs_->getLeftForeLeg()->getWorldToHipPositionInWorldFrame() + legs_->getRightForeLeg()->getWorldToHipPositionInWorldFrame())*0.5;
    const Position positionHindHipsMidPointInWorldFrame = (legs_->getLeftHindLeg()->getWorldToHipPositionInWorldFrame() + legs_->getRightHindLeg()->getWorldToHipPositionInWorldFrame())*0.5;
    Vector currentHeadingDirectionInWorldFrame = Vector(positionForeHipsMidPointInWorldFrame-positionHindHipsMidPointInWorldFrame);
    currentHeadingDirectionInWorldFrame.z() = 0.0;
    //---

    // // Yaw angle in world frame
    RotationQuaternion orientationCurrentHeadingToDesiredHeading;

    try {
      orientationCurrentHeadingToDesiredHeading.setFromVectors(currentHeadingDirectionInWorldFrame.toImplementation(),
                                                        desiredHeadingDirectionInWorldFrame.toImplementation());
    } catch (std::exception& e) {
      std::cout << e.what() << '\n';
      std::cout << "currentHeadingDirectionInWorldFrame: " << currentHeadingDirectionInWorldFrame <<std::endl;
      std::cout << "desiredHeadingDirectionInWorldFrame: " << desiredHeadingDirectionInWorldFrame <<std::endl;
      orientationCurrentHeadingToDesiredHeading.setIdentity();
    }

    //--- Compose rotations
    //desiredOrientationOffset_ = RotationQuaternion(AngleAxis(4.0*M_PI/180.0,0.0,1.0,0.0));
    RotationQuaternion orientationControlToDesiredBase = desiredOrientationOffset_*orientationCurrentHeadingToDesiredHeading;
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
