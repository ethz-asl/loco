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
  TorsoControlDynamicGaitFreePlane(legs, torso, terrain)
{
  delete comControl_;
  comControl_ = new CoMOverSupportPolygonControlStaticGait(legs, torso);

  const double defaultHeight = 0.39;
  desiredTorsoCoMHeightAboveGroundInControlFrameOffset_  = defaultHeight;
}


TorsoControlStaticGait::~TorsoControlStaticGait() {
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
  bool advanced = true;
  advanced &= TorsoControlDynamicGaitFreePlane::advance(dt);

  //  RotationQuaternion orientationWorldToHeadingOnTerrain = getOrientationWorldToHeadingOnTerrainSurface(getOrientationWorldToHeadingBasedOnHipLocations());
  //  RotationQuaternion orientationControlToHeadingOnTerrain = orientationWorldToControl*orientationWorldToHeadingOnTerrain.inverted();
  LinearVelocity linearVelocityBaseInControlFrame;
  LocalAngularVelocity angularVelocityBaseInControlFrame;
  torso_->getDesiredState().setLinearVelocityBaseInControlFrame(linearVelocityBaseInControlFrame);
  torso_->getDesiredState().setAngularVelocityBaseInControlFrame(angularVelocityBaseInControlFrame);

  return advanced;
}


bool TorsoControlStaticGait::setCoMDelta(double delta) {
  CoMOverSupportPolygonControlStaticGait* comStatic = static_cast<CoMOverSupportPolygonControlStaticGait*>(comControl_);
  comStatic->setDelta(delta);
  return true;
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
//  TiXmlHandle hDynGait(handle.FirstChild("TorsoControl").FirstChild("DynamicGait"));
//  if (!comControl_->loadParameters(hDynGait)) {
//    return false;
//  }
//  if (!loadParametersHipConfiguration(hDynGait)) {
//    return false;
//  }

  if (!TorsoControlDynamicGaitFreePlane::loadParameters(handle)) {
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
