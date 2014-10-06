/*
 * FootPlacementStrategyStaticGait.cpp
 *
 *  Created on: Oct 6, 2014
 *      Author: dario
 */


#include "loco/foot_placement_strategy/FootPlacementStrategyStaticGait.hpp"

namespace loco {


FootPlacementStrategyStaticGait::FootPlacementStrategyStaticGait(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain) :
    FootPlacementStrategyFreePlane(legs, torso, terrain)
{

  stepInterpolationFunction_.clear();
  stepInterpolationFunction_.addKnot(0, 0);
  stepInterpolationFunction_.addKnot(1.0, 1);

}


FootPlacementStrategyStaticGait::~FootPlacementStrategyStaticGait() {

}


Position FootPlacementStrategyStaticGait::getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep) {

  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();

  // Find starting point: hip projected vertically on ground
  Position positionWorldToHipOnPlaneAlongNormalInWorldFrame = leg->getStateLiftOff()->getWorldToHipPositionInWorldFrame();
  terrain_->getHeight(positionWorldToHipOnPlaneAlongNormalInWorldFrame);

  // Get offset to change between telescopic and lever configuration
  Position positionVerticalHeightOnTerrainToLeverTelescopicConfigurationInWorldFrame = getPositionVerticalHeightOnTerrainToLeverTelescopicConfigurationInWorldFrame(*leg);

  //--- Evaluate the interpolation contributions
  Position positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame = getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(*leg); //x-y
  Position positionDesiredFootOnTerrainToDesiredFootInControlFrame = getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(*leg,
                                                                                                                       positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame); // z
  //---

  //--- Build the world to desired foot position and return it
  Position positionWorldToDesiredFootInWorldFrame = positionWorldToHipOnPlaneAlongNormalInWorldFrame // starting point, hip projected on the plane along world z axis
                                                    + positionVerticalHeightOnTerrainToLeverTelescopicConfigurationInWorldFrame  // lever || telescopic component
                                                    + orientationWorldToControl.inverseRotate(positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame) // x-y
                                                    + orientationWorldToControl.inverseRotate(positionDesiredFootOnTerrainToDesiredFootInControlFrame); // z
  //---

  //--- Save for debugging
  positionWorldToHipOnPlaneAlongNormalInWorldFrame_[leg->getId()] = positionWorldToHipOnPlaneAlongNormalInWorldFrame;
  positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame_[leg->getId()] = orientationWorldToControl.inverseRotate(positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame);
  positionDesiredFootOnTerrainToDesiredFootInWorldFrame_[leg->getId()] = orientationWorldToControl.inverseRotate(positionDesiredFootOnTerrainToDesiredFootInControlFrame);
  //---

  return positionWorldToDesiredFootInWorldFrame;
  //---

}


} /* namespace loco */

