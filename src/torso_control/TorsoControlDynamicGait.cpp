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
  const double defaultHeight = 0.42;
  tValues.push_back(0.00); xValues.push_back(defaultHeight);
  tValues.push_back(0.25); xValues.push_back(defaultHeight);
  tValues.push_back(0.50); xValues.push_back(defaultHeight);
  tValues.push_back(0.75); xValues.push_back(defaultHeight);
  tValues.push_back(1.00); xValues.push_back(defaultHeight);
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
  Position lateralAndHeadingErrorInWorldFrame = comControl_.getPositionErrorVectorInWorldFrame();
  const double desiredforeHeightAboveGroundInWorldFrame = desiredTorsoForeHeightAboveGroundInWorldFrame_.evaluate(torso_->getStridePhase());
  const double desiredhindHeightAboveGroundInWorldFrame = desiredTorsoHindHeightAboveGroundInWorldFrame_.evaluate(torso_->getStridePhase());
  const double desiredMiddleHeightAboveGroundInWorldFrame = (desiredforeHeightAboveGroundInWorldFrame + desiredhindHeightAboveGroundInWorldFrame)/2.0;
  Position desiredLateralAndHeadingPositionInWorldFrame = torso_->getMeasuredState().getWorldToBasePositionInWorldFrame() + lateralAndHeadingErrorInWorldFrame;
  Position groundHeightInWorldFrame = desiredLateralAndHeadingPositionInWorldFrame;
  terrain_->getHeight(groundHeightInWorldFrame.toImplementation());
  Position desiredTorsoPositionInWorldFrame(desiredLateralAndHeadingPositionInWorldFrame.x(), desiredLateralAndHeadingPositionInWorldFrame.y(), desiredMiddleHeightAboveGroundInWorldFrame+groundHeightInWorldFrame.z());

  // pitch angle
  double height = desiredhindHeightAboveGroundInWorldFrame-desiredforeHeightAboveGroundInWorldFrame;
  double pitchAngle = atan2(height,headingDistanceFromForeToHindInBaseFrame_);

  /*RotationQuaternion desOrientationInWorldFrame(AngleAxis(pitchAngle, 0.0, 1.0, 0.0)*torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame());*/

  //Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  //RotationQuaternion desOrientationInWorldFrame = computeHeading(axis);

  RotationQuaternion desOrientationInWorldFrame = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame().inverted();
  torso_->getDesiredState().setWorldToBasePoseInWorldFrame(Pose(desiredTorsoPositionInWorldFrame, desOrientationInWorldFrame));
}

inline double safeACOS(double val){
  if (val<-1)
    return M_PI;
  if (val>1)
    return 0;
  return acos(val);
}

RotationQuaternion TorsoControlDynamicGait::computeHeading(const Eigen::Vector3d& axis) {
  RotationQuaternion rquatWorldToBase = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();
  RotationQuaternion  rquatWorldToBaseConjugated = rquatWorldToBase.conjugated();


  Eigen::Vector3d vA = rquatWorldToBaseConjugated.rotate(axis).normalized();
  Eigen::Vector3d rotAxis = -vA.cross(axis).normalized();
  double rotAngle = -safeACOS(vA.dot(axis));
  RotationQuaternion TqA = RotationQuaternion(AngleAxis(rotAngle, rotAxis));
  RotationQuaternion decomposed = TqA*rquatWorldToBaseConjugated;
  return decomposed.conjugated();

}

} /* namespace loco */
