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

  // hack:
//  desiredTorsoPositionInWorldFrame = torso_->getMeasuredState().getWorldToBasePositionInWorldFrame();
//  desiredTorsoPositionInWorldFrame.z() = 0.42;

  // pitch angle
  double height = desiredhindHeightAboveGroundInWorldFrame-desiredforeHeightAboveGroundInWorldFrame;
  double pitchAngle = atan2(height,headingDistanceFromForeToHindInBaseFrame_);

  /*RotationQuaternion desOrientationInWorldFrame(AngleAxis(pitchAngle, 0.0, 1.0, 0.0)*torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame());*/

  Eigen::Vector3d axisUp = Eigen::Vector3d::UnitZ();
  const RotationQuaternion rquatWorldToBase = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();
  RotationQuaternion desOrientationInWorldFrame = (computeHeading(rquatWorldToBase, axisUp)*RotationQuaternion());

  LinearVelocity desiredLinearVelocity(0.3,0.0,0.0);
  LocalAngularVelocity desiredAngularVelocity;

  torso_->getDesiredState().setWorldToBasePoseInWorldFrame(Pose(desiredTorsoPositionInWorldFrame, desOrientationInWorldFrame));
  torso_->getDesiredState().setBaseTwistInBaseFrame(Twist(desiredLinearVelocity, desiredAngularVelocity));
}

inline double safeACOS(double val){
  if (val<-1)
    return M_PI;
  if (val>1)
    return 0;
  return acos(val);
}

/**
  Assume that the current quaternion represents the relative orientation between two coordinate frames A and B.
  This method decomposes the current relative rotation into a twist of frame B around the axis v passed in as a
  parameter, and another more arbitrary rotation.

  AqB = AqT * TqB, where T is a frame that is obtained by rotating frame B around the axis v by the angle
  that is returned by this function.

  In the T coordinate frame, v is the same as in B, and AqT is a rotation that aligns v from A to that
  from T.

  It is assumed that vB is a unit vector!! This method returns TqB, which represents a twist about
  the axis vB.
*/
RotationQuaternion TorsoControlDynamicGait::decomposeRotation(const RotationQuaternion& AqB, const Eigen::Vector3d& vB) {

  const Eigen::Vector3d vA = AqB.inverseRotate(vB).normalized();
  Eigen::Vector3d rotAxis = (vA.cross(vB).normalized());
  rotAxis *= -1.0;
  double rotAngle = -safeACOS(vA.dot(vB));
  const RotationQuaternion TqA = RotationQuaternion(AngleAxis(rotAngle, rotAxis));
  return AqB*TqA; // TqB

}

RotationQuaternion TorsoControlDynamicGait::computeHeading(const RotationQuaternion& rquat, const Eigen::Vector3d& axis) {
  return decomposeRotation(rquat.conjugated(),axis).conjugated();

}

} /* namespace loco */
