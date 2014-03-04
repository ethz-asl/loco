/*
 * BaseControlDynamicGait.cpp
 *
 *  Created on: Feb 24, 2014
 *      Author: gech
 */

#include "loco/torso_control/TorsoControlDynamicGait.hpp"

namespace loco {

TorsoControlDynamicGait::TorsoControlDynamicGait(LegGroup* legs, TorsoBase* torso, robotTerrain::TerrainBase* terrain):
  TorsoControlBase(),
  legs_(legs),
  torso_(torso),
  terrain_(terrain),
  comControl_(legs),
  headingDistanceFromForeToHindInBaseFrame_(0.0)
{

  std::vector<double> tValues, xValues;
  tValues.push_back(0.00); xValues.push_back(0.0);
  tValues.push_back(0.25); xValues.push_back(0.0);
  tValues.push_back(0.50); xValues.push_back(0.0);
  tValues.push_back(0.75); xValues.push_back(0.0);
  tValues.push_back(1.00); xValues.push_back(0.0);
  desiredTorsoForeHeightAboveGroundInWorldFrame_.setRBFData(tValues, xValues);
  desiredTorsoHindHeightAboveGroundInWorldFrame_.setRBFData(tValues, xValues);
}

TorsoControlDynamicGait::~TorsoControlDynamicGait() {

}
bool TorsoControlDynamicGait::initialize(double dt) {
  const Position foreHipPosition = Position(legs_->getLeg(0)->getWorldToHipPositionInBaseFrame());
  const Position hindHipPosition =  Position(legs_->getLeg(2)->getWorldToHipPositionInBaseFrame());
  headingDistanceFromForeToHindInBaseFrame_ = foreHipPosition.x()-hindHipPosition.x();
//  std::cout << "head dist: " << headingDistanceFromForeToHindInBaseFrame_ << std::endl;

  return true;
}

void TorsoControlDynamicGait::advance(double dt) {
  Eigen::Vector3d lateralAndHeadingErrorInWorldFrame = comControl_.getPositionErrorVectorInWorldFrame();
  const double desiredforeHeightAboveGroundInWorldFrame = desiredTorsoForeHeightAboveGroundInWorldFrame_.evaluate(torso_->getStridePhase());
  const double desiredhindHeightAboveGroundInWorldFrame = desiredTorsoHindHeightAboveGroundInWorldFrame_.evaluate(torso_->getStridePhase());
  const double desiredMiddleHeightAboveGroundInWorldFrame = (desiredforeHeightAboveGroundInWorldFrame + desiredhindHeightAboveGroundInWorldFrame)/2.0;
  Eigen::Vector3d desiredLateralAndHeadingPositionInWorldFrame = torso_->getMeasuredState().getWorldToBasePoseInWorldFrame().getRotation().inverseRotate(torso_->getMeasuredState().getWorldToBasePoseInWorldFrame().getPosition().toImplementation()) + lateralAndHeadingErrorInWorldFrame;
  Eigen::Vector3d groundHeightInWorldFrame = desiredLateralAndHeadingPositionInWorldFrame;
  terrain_->getHeight(groundHeightInWorldFrame);
  Eigen::Vector3d desiredTorsoPositionInWorldFrame(desiredLateralAndHeadingPositionInWorldFrame.x(), desiredLateralAndHeadingPositionInWorldFrame.y(), desiredMiddleHeightAboveGroundInWorldFrame+groundHeightInWorldFrame.z());

  // pitch angle
  Position desPositionInWorldFrame(desiredTorsoPositionInWorldFrame);
  double height = desiredhindHeightAboveGroundInWorldFrame-desiredforeHeightAboveGroundInWorldFrame;
  double pitchAngle = atan2(height,headingDistanceFromForeToHindInBaseFrame_);

  RotationQuaternion desOrientationInWorldFrame(AngleAxis(pitchAngle, 0.0, 1.0, 0.0)*torso_->getDesiredState().getWorldToBaseOrientationInWorldFrame());

  torso_->getDesiredState().setWorldToBasePoseInWorldFrame(Pose(desPositionInWorldFrame, desOrientationInWorldFrame));
}

} /* namespace loco */
