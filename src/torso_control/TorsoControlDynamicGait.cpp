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
  headingDistanceFromForeToHindInBasFrame_(0.0)
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

void TorsoControlDynamicGait::advance(double dt) {
  Eigen::Vector3d lateralAndHeadingErrorInWorldFrame = comControl_.getPositionErrorVectorInWorldFrame();
  const double desiredforeHeightAboveGroundInWorldFrame = desiredTorsoForeHeightAboveGroundInWorldFrame_.evaluate(torso_->getStridePhase());
  const double desiredhindHeightAboveGroundInWorldFrame = desiredTorsoHindHeightAboveGroundInWorldFrame_.evaluate(torso_->getStridePhase());
  const double desiredMiddleHeightAboveGroundInWorldFrame = (desiredforeHeightAboveGroundInWorldFrame + desiredhindHeightAboveGroundInWorldFrame)/2.0;
  Eigen::Vector3d desiredLateralAndHeadingPositionInWorldFrame = torso_->getMeasuredPoseInWorldFrame().getRotation().inverseRotate(torso_->getMeasuredPoseInWorldFrame().getPosition().toImplementation()) + lateralAndHeadingErrorInWorldFrame;
  Eigen::Vector3d groundHeightInWorldFrame = desiredLateralAndHeadingPositionInWorldFrame;
  terrain_->getHeight(groundHeightInWorldFrame);
  Eigen::Vector3d desiredTorsoPositionInWorldFrame(desiredLateralAndHeadingPositionInWorldFrame.x(), desiredLateralAndHeadingPositionInWorldFrame.y(), desiredMiddleHeightAboveGroundInWorldFrame+groundHeightInWorldFrame.z());

  // pitch angle
  TorsoBase::Pose::Position desPositionInWorldFrame(desiredTorsoPositionInWorldFrame);
  double height = desiredhindHeightAboveGroundInWorldFrame-desiredforeHeightAboveGroundInWorldFrame;
  double pitchAngle = atan2(height,headingDistanceFromForeToHindInBasFrame_);

  TorsoBase::Pose::Rotation desOrientationInWorldFrame(AngleAxis(pitchAngle, 0.0, 1.0, 0.0)*torso_->getDesiredPoseInWorldFrame().getRotation());

  torso_->setDesiredPoseInWorldFrame(TorsoBase::Pose(desPositionInWorldFrame, desOrientationInWorldFrame));
}

} /* namespace loco */
