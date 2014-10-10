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
  comControl_ = new CoMOverSupportPolygonControlStaticGait(legs, torso);
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


} /* namespace loco */
