/*****************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Christian Gehring, Péter Fankhauser, C. Dario Bellicoso, Stelian Coros
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
 * TorsoControlDynamicGaitFreePlane.cpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Dario Bellicoso
 */

#include "loco/torso_control/TorsoControlDynamicGaitFreePlane.hpp"
#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlDynamicGait.hpp"
#include "loco/temp_helpers/math.hpp"
#include <exception>

namespace loco {

TorsoControlDynamicGaitFreePlane::TorsoControlDynamicGaitFreePlane(LegGroup* legs, TorsoBase* torso,  TerrainModelBase* terrain):
  TorsoControlGaitContainer(legs, torso, terrain),
  maxDesiredPitchRadians_(5.0*M_PI/180.0),
  desiredPitchSlope_(1.0),
  maxDesiredRollRadians_(5.0*M_PI/180.0),
  desiredRollSlope_(1.0),
  adaptToTerrain_(CompleteAdaption)
{
  firstOrderFilter_ = new robotUtils::FirstOrderFilter();
  comControl_ = new CoMOverSupportPolygonControlDynamicGait(legs_);
}


TorsoControlDynamicGaitFreePlane::~TorsoControlDynamicGaitFreePlane() {
  if (firstOrderFilter_) delete firstOrderFilter_;
  delete comControl_;
}


bool TorsoControlDynamicGaitFreePlane::initialize(double dt) {
  const Position foreHipPosition = legs_->getLeg(0)->getPositionWorldToHipInBaseFrame();
  const Position hindHipPosition = legs_->getLeg(2)->getPositionWorldToHipInBaseFrame();
  headingDistanceFromForeToHindInBaseFrame_ = foreHipPosition.x()-hindHipPosition.x();

  firstOrderFilter_->initialize(0.0, 1.0, 1.0);

  return true;
}


bool TorsoControlDynamicGaitFreePlane::advance(double dt) {
  comControl_->advance(dt);

  // Get measured orientation
  const RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl(); // --> current heading orientation

  /********************************************************************************************************
   * Set desired base position in world frame
   ********************************************************************************************************/
  // this is the position we need to compute:
  Position positionControlToTargetBaseInControlFrame;


  /* Compute the horizontal component of the desired position in world frame.
   *
   *  evaluate desired CoM position in control frame
   */
  Position positionWorldToDesiredHorizontalBaseInWorldFrame = comControl_->getPositionWorldToDesiredCoMInWorldFrame();

  // this is the desired location of the base location relative to the origin of the control frame projected on the x-y plane of the world frame and expressed in the world frame
  Position positionHorizontalControlToHorizontalBaseInWorldFrame = positionWorldToDesiredHorizontalBaseInWorldFrame
                                                                   - torso_->getMeasuredState().getPositionWorldToControlInWorldFrame();
  positionHorizontalControlToHorizontalBaseInWorldFrame.z() = 0.0;



  /*************************************************************************
   *  Method I - NEW
   *
   * The desired base position has to lie on the line:
   *    W_r_C_B* = W_r_C_Bh* + lambda*W_e_z^W
   * and on the plane parallel to the surface (with normal W_n) with
   * the desired height above ground (h*):
   *    C_n * (C_r_C_B* - C_n * h*) = 0
   *
   *  The intersection point is given by:
   *                             C_n * ( C_n * h* - C_r_C_Bh*)
   *    C_r_C_B* = C_r_C_Bh*  +  ----------------------------- * C_e_z^W
   *                                     C_n * C_e_z^W
   *
   *   We compute it in control frame instead of world frame.
   *
   *************************************************************************/
//    // this is the surface normal
//    loco::Vector surfaceNormalInWorldFrame;
//    terrain_->getNormal(loco::Position::Zero(), surfaceNormalInWorldFrame);
//    loco::Vector surfaceNormalInControlFrame = orientationWorldToControl.rotate(surfaceNormalInWorldFrame);
//
//    const loco::Vector verticalAxisOfWorldFrameInWorldFrame = loco::Vector::UnitZ();
//    const loco::Vector verticalAxisOfWorldFrameInControlFrame = orientationWorldToControl.rotate(verticalAxisOfWorldFrameInWorldFrame);
//
//
//    const Position positionHorizontalControlToHorizontalBaseInControlFrame = orientationWorldToControl.rotate(positionHorizontalControlToHorizontalBaseInWorldFrame);
//
//    Position temp2 = (desiredTorsoCoMHeightAboveGroundInControlFrameOffset_*Position(surfaceNormalInControlFrame))
//           - positionHorizontalControlToHorizontalBaseInControlFrame;
//    double scaleNumerator = Position(surfaceNormalInControlFrame).dot(temp2);
//    double scaleDenominator = Position(surfaceNormalInControlFrame).dot(verticalAxisOfWorldFrameInControlFrame);
//    double scale = scaleNumerator/scaleDenominator;
//
//    positionControlToTargetBaseInControlFrame = orientationWorldToControl.rotate(positionHorizontalControlToHorizontalBaseInWorldFrame)
//                                                         + Position(verticalAxisOfWorldFrameInControlFrame*scale);

  /*************************************************************************
  *  Method II - OLD
  *
  * The desired base position has to lie on the line:
  *    W_r_C_B* = W_r_C_Bh* + lambda*W_e_z^W
  *
  * and the vertical component in world frame has to be equal to the sum of
  * the height of the terrain where this line goes through and the desired
  * height of the torso above ground.
  *
  *************************************************************************/

  RotationQuaternion orientationWorldToTerrain = getOrientationWorldToHeadingOnTerrainSurface(RotationQuaternion());
  Position positionWorldToDesiredHeightAboveTerrainInTerrainFrame(0.0, 0.0, desiredTorsoCoMHeightAboveGroundInControlFrameOffset_);
  Position positionWorldToDesiredHeightAboveTerrainInWorldFrame = orientationWorldToTerrain.inverseRotate(positionWorldToDesiredHeightAboveTerrainInTerrainFrame);

  loco::Vector surfaceNormalInWorldFrame;
  terrain_->getNormal(loco::Position::Zero(), surfaceNormalInWorldFrame);
  double heightOverTerrain = positionWorldToDesiredHeightAboveTerrainInWorldFrame.dot(surfaceNormalInWorldFrame);
  heightOverTerrain /= surfaceNormalInWorldFrame.z();

  double heightOfTerrainInWorldFrame = 0.0;
  terrain_->getHeight(positionWorldToDesiredHorizontalBaseInWorldFrame, heightOfTerrainInWorldFrame);

//  Position positionWorldToHorizontalBaseInWorldFrame_temp = positionWorldToDesiredHorizontalBaseInWorldFrame + heightOfTerrainInWorldFrame*Position::UnitZ();

  Position positionControlToTargetBaseInWorldFrame = positionHorizontalControlToHorizontalBaseInWorldFrame
                                              + (heightOfTerrainInWorldFrame + heightOverTerrain)*Position::UnitZ();
  positionControlToTargetBaseInControlFrame = orientationWorldToControl.rotate(positionControlToTargetBaseInWorldFrame + desiredPositionOffsetInWorldFrame_);

  /********************************************************************************************************
   * End set desired CoM position in world frame *
   ********************************************************************************************************/


  /********************************************************************************************************
   * Set the desired orientation of the base frame with respect to the control frame
   ********************************************************************************************************/
  // this is the orientation we need to compute
  RotationQuaternion orientationControlToDesiredBase;

  /*************************************************************************
   * Method I - NEW
   *
   * Assumes that the control frame is aligned with the terrain surface.
   * If the orientation of the body should be fully adapted to the terrain,
   * we don't have to do anything.
   *
   * The desired orientation will be given by a composition of rotations:
   *    --> rotation from world to current heading (starting point for the composition).
   *    --> rotation from current heading to desired heading. This can be due to the body yaw induced by the foot holds.
   *    --> rotation from desired heading to desired base. This is an adaption to the terrain's estimated pitch and roll angles.
   *
   *************************************************************************/
  RotationQuaternion orientationCurrentHeadingToDesiredHeading = getOrientationHeadingToDesiredHeadingBasedOnFeetLocations(positionWorldToDesiredHorizontalBaseInWorldFrame);

  //--- Compose rotations
  orientationControlToDesiredBase = desiredOrientationOffset_*orientationCurrentHeadingToDesiredHeading;
  //---


  /*************************************************************************
   * Method II - OLD
   *
   *************************************************************************/
//  RotationQuaternion orientationCurrentHeadingToDesiredHeading = getOrientationHeadingToDesiredHeadingBasedOnFeetLocations(positionWorldToDesiredHorizontalBaseInWorldFrame);
//  RotationQuaternion orientationWorldToDesiredBase =  getOrientationWorldToHeadingOnTerrainSurface(orientationCurrentHeadingToDesiredHeading*getOrientationWorldToHeadingBasedOnHipLocations());
//  orientationControlToDesiredBase = orientationWorldToDesiredBase*orientationWorldToControl.inverted();

  /*******************************
   * End set desired orientation *
   *******************************/


  torso_->getDesiredState().setPositionControlToBaseInControlFrame(positionControlToTargetBaseInControlFrame);
  torso_->getDesiredState().setOrientationControlToBase(orientationControlToDesiredBase);

//  EulerAnglesZyx torsoAttitude = EulerAnglesZyx(orientationControlToDesiredBase).getUnique();
//  std::cout << "*******" << std::endl;
//  std::cout << "Desired torso position in control frame: " << std::endl << positionControlToTargetBaseInControlFrame << std::endl;
//  std::cout << "Desired torso attitude in control frame: " << std::endl << torsoAttitude.roll() << " "
//                                                                        << torsoAttitude.pitch() << " "
//                                                                        << torsoAttitude.yaw() << std::endl;
//  std::cout << "*******" << std::endl << std::endl;

  /*************************************************************************
   * Method II - OLD
   * The input  torso_->getDesiredState().setLinearVelocityBaseInControlFrame is actually not in control frame!!!!
   * We need to correct this with a hack (fixme in future)
   *
   * velocities are given in frame HeadingOnTerrain
   *
   *************************************************************************/
//  RotationQuaternion orientationWorldToHeadingOnTerrain = getOrientationWorldToHeadingOnTerrainSurface(getOrientationWorldToHeadingBasedOnHipLocations());
//  RotationQuaternion orientationControlToHeadingOnTerrain = orientationWorldToControl*orientationWorldToHeadingOnTerrain.inverted();
//          LinearVelocity linearVelocityBaseInControlFrame = orientationControlToHeadingOnTerrain.rotate(torso_->getDesiredState().getLinearVelocityBaseInControlFrame());
//  torso_->getDesiredState().setLinearVelocityBaseInControlFrame(linearVelocityBaseInControlFrame);
  return true;
}


bool TorsoControlDynamicGaitFreePlane::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle handleTorsoConfiguration(handle.FirstChild("TorsoControl").FirstChild("TorsoConfiguration"));
  TiXmlHandle handleDynamicGait(handle.FirstChild("TorsoControl").FirstChild("DynamicGait"));

  if (!comControl_->loadParameters(handleDynamicGait)) {
    return false;
  }

  if (!loadParametersTorsoConfiguration(handleTorsoConfiguration)) {
    return false;
  }

  if (!loadParametersHipConfiguration(handleDynamicGait)) {
    return false;
  }

  return true;
}


bool TorsoControlDynamicGaitFreePlane::setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t) {
  const TorsoControlDynamicGaitFreePlane& controller1 = static_cast<const TorsoControlDynamicGaitFreePlane&>(torsoController1);
  const TorsoControlDynamicGaitFreePlane& controller2 = static_cast<const TorsoControlDynamicGaitFreePlane&>(torsoController2);

  this->comControl_->setToInterpolated(controller1.getCoMOverSupportPolygonControl(), controller2.getCoMOverSupportPolygonControl(), t);
  desiredTorsoForeHeightAboveGroundInWorldFrameOffset_ = linearlyInterpolate(controller1.getDesiredTorsoForeHeightAboveGroundInWorldFrameOffset(),
                                                                             controller2.getDesiredTorsoForeHeightAboveGroundInWorldFrameOffset(),
                                                                             0.0,
                                                                             1.0,
                                                                             t);
  desiredTorsoHindHeightAboveGroundInWorldFrameOffset_ = linearlyInterpolate(controller1.getDesiredTorsoHindHeightAboveGroundInWorldFrameOffset(),
                                                                             controller2.getDesiredTorsoHindHeightAboveGroundInWorldFrameOffset(),
                                                                             0.0,
                                                                             1.0,
                                                                             t);
  desiredTorsoCoMHeightAboveGroundInControlFrameOffset_ = linearlyInterpolate(controller1.getDesiredTorsoCoMHeightAboveGroundInControlFrameOffset(),
                                                                              controller2.getDesiredTorsoCoMHeightAboveGroundInControlFrameOffset(),
                                                                              0.0,
                                                                              1.0,
                                                                              t);

  if(!interpolateHeightTrajectory(desiredTorsoForeHeightAboveGroundInWorldFrame_, controller1.desiredTorsoForeHeightAboveGroundInWorldFrame_, controller2.desiredTorsoForeHeightAboveGroundInWorldFrame_, t)) {
    return false;
  }
  if(!interpolateHeightTrajectory(desiredTorsoHindHeightAboveGroundInWorldFrame_, controller1.desiredTorsoHindHeightAboveGroundInWorldFrame_, controller2.desiredTorsoHindHeightAboveGroundInWorldFrame_, t)) {
    return false;
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
