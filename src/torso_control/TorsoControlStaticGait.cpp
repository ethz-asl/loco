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

  CoMOverSupportPolygonControlStaticGait* comStatic = (CoMOverSupportPolygonControlStaticGait*)comControl_;
  comStatic->initialize();

  return true;
}

bool TorsoControlStaticGait::advance(double dt) {
  TorsoControlDynamicGaitFreePlane::advance(dt);

  //  RotationQuaternion orientationWorldToHeadingOnTerrain = getOrientationWorldToHeadingOnTerrainSurface(getOrientationWorldToHeadingBasedOnHipLocations());
  //  RotationQuaternion orientationControlToHeadingOnTerrain = orientationWorldToControl*orientationWorldToHeadingOnTerrain.inverted();
  LinearVelocity linearVelocityBaseInControlFrame;
  LocalAngularVelocity angularVelocityBaseInControlFrame;
  torso_->getDesiredState().setLinearVelocityBaseInControlFrame(linearVelocityBaseInControlFrame);
  torso_->getDesiredState().setAngularVelocityBaseInControlFrame(angularVelocityBaseInControlFrame);

}

bool TorsoControlStaticGait::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle hDynGait(handle.FirstChild("TorsoControl").FirstChild("DynamicGait"));
  if (!comControl_->loadParameters(hDynGait)) {
    return false;
  }
  if (!loadParametersHipConfiguration(hDynGait)) {
    return false;
  }

  TiXmlHandle hStGait(handle.FirstChild("TorsoControl").FirstChild("StaticGait"));
  if (!loadParametersTorsoConfiguration(hStGait)) {
    return false;
  }

  return true;
}


bool TorsoControlStaticGait::loadParametersTorsoConfiguration(const TiXmlHandle& hParameterSet) {

  TiXmlElement* pElem;

  // Check if "TorsoConfiguration" exists in parameter file
  pElem = hParameterSet.FirstChild("TorsoConfiguration").Element();
  if (!pElem) {
    printf("Could not find TorsoConfiguration\n");
    return false;
  }

  TiXmlElement* child = hParameterSet.FirstChild("TorsoConfiguration").FirstChild().ToElement();
  for(; child; child=child->NextSiblingElement()) {
    // If "TorsoHeight" element is found, try to read "torsoHeight" value
    if (child->ValueStr().compare("TorsoHeight") == 0) {
      bool isFore = false;
      bool isHind = false;
      double defaultTorsoHeight = 0.0;
      if (child->QueryDoubleAttribute("torsoHeight", &defaultTorsoHeight)!=TIXML_SUCCESS) {
        printf("Could not find offset!\n");
      }
      else {
        desiredTorsoCoMHeightAboveGroundInControlFrameOffset_ = defaultTorsoHeight;
      }
    }

  }

  return true;
}


} /* namespace loco */
