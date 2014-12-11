/********************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014,  C. Dario Bellicoso, Christian Gehring, Péter Fankhauser, Stelian Coros
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
/*!
* @file     CoMOverSupportPolygonControlLeverConfiguration.hpp
* @author   C. Dario Bellicoso
* @date     Oct 7, 2014
* @brief
*/
#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlLeverConfiguration.hpp"
#include "loco/common/TerrainModelFreePlane.hpp"

namespace loco {

CoMOverSupportPolygonControlLeverConfiguration::CoMOverSupportPolygonControlLeverConfiguration(LegGroup* legs, TorsoBase* torso, TerrainModelBase* terrainModel):
    CoMOverSupportPolygonControlBase(legs),
    torso_(torso),
    terrainModel_(terrainModel)
{
  positionCenterToForeHindSupportFeetInControlFrame_[0] = Position::Zero();
  positionCenterToForeHindSupportFeetInControlFrame_[1] = Position::Zero();
  positionWorldToCenterInWorldFrame_ = Position::Zero();
}


CoMOverSupportPolygonControlLeverConfiguration::~CoMOverSupportPolygonControlLeverConfiguration() {

}

bool CoMOverSupportPolygonControlLeverConfiguration::setToInterpolated(const CoMOverSupportPolygonControlBase& supportPolygon1, const CoMOverSupportPolygonControlBase& supportPolygon2, double t) {
  return false;
}


void CoMOverSupportPolygonControlLeverConfiguration::advance(double dt) {

  Position comTarget;

  // Matlab code
//  % Find ratios - control vertical
//  ratio_x_controlvertical = abs((rb_onplane_worldvertical(1) - rf_lf_controlvertical(1))/...
//                            (rb_onplane_worldvertical(1) - rf_lh_controlvertical(1)));
//  ratio_y_controlvertical = abs((rb_onplane_worldvertical(2) - rf_lf_controlvertical(2))/...
//                            (rb_onplane_worldvertical(2) - rf_rf_controlvertical(2)));
//
//  shift_y = hipToHipY*ratio_x_controlvertical/(1+ratio_x_controlvertical) - rf_lf_controlvertical(2);

  RotationQuaternion orientationWorldToBase = torso_->getMeasuredState().getOrientationWorldToBase();
  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();

  TerrainModelFreePlane* terrainModel = (TerrainModelFreePlane*)terrainModel_;

  Position rb_onplane_worldvertical = torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
  terrainModel->getHeight(rb_onplane_worldvertical);

  Position rb_onplane_controlvertical = terrainModel->getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame());
//  std::cout << "rb onplane control: " << rb_onplane_controlvertical << std::endl;
//  std::cout << "rb onplane world: " << rb_onplane_worldvertical << std::endl;

  positionWorldToCenterInWorldFrame_ = rb_onplane_controlvertical;

  Position positionBaseOnTerrainToForeSupportLegInControlFrame;
  Position positionBaseOnTerrainToHindSupportLegInControlFrame;

  // Get contact points
  if (legs_->getLeftForeLeg()->isSupportLeg()) {
    positionBaseOnTerrainToForeSupportLegInControlFrame = terrainModel->getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(legs_->getLeftForeLeg()->getPositionWorldToFootInWorldFrame());
    positionBaseOnTerrainToForeSupportLegInControlFrame = orientationWorldToControl.rotate(positionBaseOnTerrainToForeSupportLegInControlFrame-rb_onplane_controlvertical);

    positionBaseOnTerrainToHindSupportLegInControlFrame = terrainModel->getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(legs_->getRightHindLeg()->getPositionWorldToFootInWorldFrame());
    positionBaseOnTerrainToHindSupportLegInControlFrame = orientationWorldToControl.rotate(positionBaseOnTerrainToHindSupportLegInControlFrame-rb_onplane_controlvertical);

    positionCenterToForeHindSupportFeetInControlFrame_[0] = positionBaseOnTerrainToForeSupportLegInControlFrame;
    positionCenterToForeHindSupportFeetInControlFrame_[1] = positionBaseOnTerrainToHindSupportLegInControlFrame;
  }
  else {
    positionBaseOnTerrainToForeSupportLegInControlFrame = terrainModel->getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(legs_->getRightForeLeg()->getPositionWorldToFootInWorldFrame());
    positionBaseOnTerrainToForeSupportLegInControlFrame = orientationWorldToControl.rotate(positionBaseOnTerrainToForeSupportLegInControlFrame-rb_onplane_controlvertical);

    positionBaseOnTerrainToHindSupportLegInControlFrame = terrainModel->getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(legs_->getLeftHindLeg()->getPositionWorldToFootInWorldFrame());
    positionBaseOnTerrainToHindSupportLegInControlFrame = orientationWorldToControl.rotate(legs_->getLeftHindLeg()->getPositionWorldToFootInWorldFrame()-rb_onplane_controlvertical);

    positionCenterToForeHindSupportFeetInControlFrame_[0] = positionBaseOnTerrainToForeSupportLegInControlFrame;
    positionCenterToForeHindSupportFeetInControlFrame_[1] = positionBaseOnTerrainToHindSupportLegInControlFrame;

  }

  Position r_controlToBaseOnTerrainInControlFrame = orientationWorldToControl.rotate(rb_onplane_worldvertical - rb_onplane_controlvertical);

  double ratioX = fabs(r_controlToBaseOnTerrainInControlFrame.x() - positionBaseOnTerrainToForeSupportLegInControlFrame.x())
                  /
                  fabs(r_controlToBaseOnTerrainInControlFrame.x() - positionBaseOnTerrainToHindSupportLegInControlFrame.x());

  double hipToHipX = fabs(positionBaseOnTerrainToForeSupportLegInControlFrame.x() - positionBaseOnTerrainToHindSupportLegInControlFrame.x());
  double hipToHipY = fabs(positionBaseOnTerrainToForeSupportLegInControlFrame.y() - positionBaseOnTerrainToHindSupportLegInControlFrame.y());

  comTarget = r_controlToBaseOnTerrainInControlFrame;

  double ly_2 = 0.0;
  double ratioY = 0.0;

  if (legs_->getLeftForeLeg()->isSupportLeg()) {
    ly_2 = hipToHipY/(1.0+ratioX);
    comTarget.y() = - (hipToHipY*0.5 - ly_2);
  }
  else {
    ly_2 = hipToHipY*ratioX/(1.0+ratioX);
    comTarget.y() = ly_2 - hipToHipY*0.5;
  }

  Position aux = rb_onplane_controlvertical + orientationWorldToControl.inverseRotate(comTarget);
  comTarget = aux;

  ratioY = fabs(comTarget.y() - positionBaseOnTerrainToForeSupportLegInControlFrame.y())
           /
           fabs(comTarget.y() - positionBaseOnTerrainToHindSupportLegInControlFrame.y());

//  std::cout << "check ratios" << std::endl
//            << "x: " << ratioX << std::endl
//            << "y: " << ratioY << std::endl
//            << "ly2/ly1: " << ly_2/(hipToHipY-ly_2) << std::endl
//            << "ly1/ly2: " << 1.0/(ly_2/(hipToHipY-ly_2)) << std::endl
//            << "com: " << comTarget << std::endl
//            << "fore support" << positionBaseOnTerrainToForeSupportLegInControlFrame << std::endl;

  positionWorldToDesiredCoMInWorldFrame_ = comTarget + Position(headingOffset_, lateralOffset_, 0.0);
  positionWorldToDesiredCoMInWorldFrame_.z() = 0.0;

}

} /* namespace loco */
