/*
 * TorsoControlStaticGait.cpp
 *
 *  Created on: Oct 9, 2014
 *      Author: dario
 */

#include "loco/torso_control/TorsoControlStaticGait.hpp"
#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlStaticGait.hpp"


namespace loco {


TorsoControlStaticGait::TorsoControlStaticGait(LegGroup* legs, TorsoBase* torso, TerrainModelBase* terrain):
  TorsoControlGaitContainer(legs, torso, terrain),
  isInStandConfiguration_(false)
{
  comControl_ = new CoMOverSupportPolygonControlStaticGait(legs, torso);
}


TorsoControlStaticGait::~TorsoControlStaticGait() {
  delete comControl_;
}


bool TorsoControlStaticGait::initialize(double dt) {
  const Position foreHipPosition = legs_->getLeg(0)->getPositionWorldToHipInBaseFrame();
  const Position hindHipPosition = legs_->getLeg(2)->getPositionWorldToHipInBaseFrame();
  headingDistanceFromForeToHindInBaseFrame_ = foreHipPosition.x()-hindHipPosition.x();

  CoMOverSupportPolygonControlStaticGait* comStatic = static_cast<CoMOverSupportPolygonControlStaticGait*>(comControl_);
  comStatic->initialize();

  return true;
}


bool TorsoControlStaticGait::advance(double dt) {
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

  LinearVelocity linearVelocityBaseInControlFrame;
  LocalAngularVelocity angularVelocityBaseInControlFrame;
  torso_->getDesiredState().setLinearVelocityBaseInControlFrame(linearVelocityBaseInControlFrame);
  torso_->getDesiredState().setAngularVelocityBaseInControlFrame(angularVelocityBaseInControlFrame);

  return true;
}


bool TorsoControlStaticGait::setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t) {
  return false;
}


void TorsoControlStaticGait::setIsInStandConfiguration(bool isInStandConfiguration) {
  isInStandConfiguration_ = isInStandConfiguration;
  CoMOverSupportPolygonControlStaticGait* comStatic = static_cast<CoMOverSupportPolygonControlStaticGait*>(comControl_);
  comStatic->setIsInStandConfiguration(isInStandConfiguration);
}


bool TorsoControlStaticGait::getIsInStandConfiguration() const {
  return isInStandConfiguration_;
}


bool TorsoControlStaticGait::loadParameters(const TiXmlHandle& handle) {

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

  TiXmlHandle hStGait(handle.FirstChild("TorsoControl").FirstChild("StaticGait"));
  CoMOverSupportPolygonControlStaticGait* staticComControl = static_cast<CoMOverSupportPolygonControlStaticGait*>(comControl_);
  if (!staticComControl->loadParametersStaticGait(hStGait)) {
    return false;
  }

  return true;
}


} /* namespace loco */
