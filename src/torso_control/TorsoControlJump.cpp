/*
 * TorsoControlJump.cpp
 *
 *  Created on: Jun 19, 2014
 *      Author: wko
 */

#include "loco/torso_control/TorsoControlJump.hpp"
#include <exception>
namespace loco {

TorsoControlJump::TorsoControlJump(LegGroup* legs, TorsoBase* torso,
                                   loco::TerrainModelBase* terrain)
    : TorsoControlBase(),
      legs_(legs),
      torso_(torso),
      terrain_(terrain),
      comControl_(legs),
      headingDistanceFromForeToHindInBaseFrame_(0.0),
      currentTime_(0.0) {
}

TorsoControlJump::~TorsoControlJump() {
//  output_.close();
}

bool TorsoControlJump::initialize(double dt) {
  const Position foreHipPosition = legs_->getLeg(0)
      ->getWorldToHipPositionInBaseFrame();
  const Position hindHipPosition = legs_->getLeg(2)
      ->getWorldToHipPositionInBaseFrame();
  headingDistanceFromForeToHindInBaseFrame_ = foreHipPosition.x()
      - hindHipPosition.x();

  // Make sure the robot is on the ground and did not jump before
  if (legs_->getLeftForeLeg()->isGrounded()
      && legs_->getRightForeLeg()->isGrounded()
      && legs_->getLeftHindLeg()->isGrounded()
      && legs_->getRightForeLeg()->isGrounded()
      && state_ != State::APEX) {
    state_ = State::INIT;

    output_.open("output.dat");
    input_.open("input.dat");
  }

  return true;
}

/**
 * Returns phase of execution.
 */
State TorsoControlJump::getState() {
  return state_;
}

/**
 * Sets GaussianKernelJumpPropagator to follow.
 */
void TorsoControlJump::setTrajectoryFollower(GaussianKernelJumpPropagator trajectoryFollower) {
  trajectoryFollower_ = trajectoryFollower;
}

/**
 * Enables body jump goal tracking if set.
 */
void TorsoControlJump::setInTorsoPositionMode (bool isInTorsoPositionMode) {
  inTorsoPositionMode_ = isInTorsoPositionMode;
}

void TorsoControlJump::advance(double dt) {
  comControl_.advance(dt);
  const RotationQuaternion orientationWorldToHeading =
      torso_->getMeasuredState().getWorldToHeadingOrientation();

  Position lateralAndHeadingPositionInWorldFrame = comControl_
      .getDesiredWorldToCoMPositionInWorldFrame();

  Position desiredLateralAndHeadingPositionInWorldFrame =
      lateralAndHeadingPositionInWorldFrame;
  Position groundHeightInWorldFrame =
      desiredLateralAndHeadingPositionInWorldFrame;

  terrain_->getHeight(groundHeightInWorldFrame);

  double desiredTorsoHeightAboveGroundInWorldFrame;
  updateState();

  if (inTorsoPositionMode_) {
    if (state_ == State::LIFTOFF || state_ == State::INIT) {
      desiredTorsoHeightAboveGroundInWorldFrame = trajectoryFollower_.predict(
          0);

      input_ << currentTime_ << " " << desiredTorsoHeightAboveGroundInWorldFrame
             << std::endl;
    } else if (state_ == State::APEX || state_ == State::TOUCHDOWN) {
      desiredTorsoHeightAboveGroundInWorldFrame = 0.42;
    }

    Position desiredTorsoPositionInWorldFrame(
        desiredLateralAndHeadingPositionInWorldFrame.x(),
        desiredLateralAndHeadingPositionInWorldFrame.y(),
        desiredTorsoHeightAboveGroundInWorldFrame
            + groundHeightInWorldFrame.z());

    //std::cout << desiredTorsoHeightAboveGroundInWorldFrame << std::endl;

    /* --- desired orientation --- */
    // Just keep torso parallel to ground for now
    RotationQuaternion desOrientationWorldToBase = RotationQuaternion();

    /* --- end desired orientation --- */
    torso_->getDesiredState().setWorldToBasePoseInWorldFrame(
        Pose(desiredTorsoPositionInWorldFrame, desOrientationWorldToBase));
  }
  addMeasuresToTrajectory(
      torso_->getMeasuredState().getWorldToBasePositionInWorldFrame().z());

  /* Output jump trajectory to file */
  output_ << currentTime << " "
          << torso_->getMeasuredState().getWorldToBasePositionInWorldFrame().z()
          << std::endl;
  currentTime += dt;
}

/**
 * Keeps track of current state of jump.
 * Jump occurs in the following phases:
 * INIT -> LIFTOFF -> APEX -> TOUCHDOWN
 */
void TorsoControlJump::updateState() {
  if (legs_->getLeftForeLeg()->getStateLiftOff()->isNow()
      && legs_->getRightForeLeg()->getStateLiftOff()->isNow()
      && legs_->getLeftHindLeg()->getStateLiftOff()->isNow()
      && legs_->getRightHindLeg()->getStateLiftOff()->isNow()
      && state_ == State::INIT) {

    state_ = State::LIFTOFF;

  } else if (!legs_->getLeftForeLeg()->isGrounded()
      && !legs_->getLeftHindLeg()->isGrounded()
      && !legs_->getRightHindLeg()->isGrounded()
      && !legs_->getRightForeLeg()->isGrounded() && state_ == State::LIFTOFF) {

    state_ = State::APEX;

  } else if (legs_->getLeftForeLeg()->isGrounded()
      && legs_->getRightForeLeg()->isGrounded()
      && legs_->getLeftHindLeg()->isGrounded()
      && legs_->getRightForeLeg()->isGrounded() && state_ == State::APEX) {

    state_ = State::TOUCHDOWN;
  }
}

void TorsoControlJump::addMeasuresToTrajectory(double baseHeight) {
  measuredHeightTrajectory_.push_back(baseHeight);

  leftFrontContactFlagTrajectory_.push_back(
      legs_->getLeftForeLeg()->isGrounded());
  leftHindContactFlagTrajectory_.push_back(
      legs_->getLeftHindLeg()->isGrounded());
  rightHindContactFlagTrajectory_.push_back(
      legs_->getRightHindLeg()->isGrounded());
  rightFrontContactFlagTrajectory_.push_back(
      legs_->getRightForeLeg()->isGrounded());
}

std::vector<bool> TorsoControlJump::getMeasuredContactFlags(Leg leg) {
  switch (leg) {
    case FRONT_LEFT:
      return leftFrontContactFlagTrajectory_;
    case FRONT_RIGHT:
      return leftFrontContactFlagTrajectory_;
    case HIND_LEFT:
      return leftFrontContactFlagTrajectory_;
    case HIND_RIGHT:
      return leftFrontContactFlagTrajectory_;
    default:
      std::vector<bool> emptyVector;
      return emptyVector;
  }
}

std::vector<double> TorsoControlJump::getMeasuredTrajectory() {
  return measuredHeightTrajectory_;
}

inline double safeACOS(double val) {
  if (val < -1)
    return M_PI;
  if (val > 1)
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
RotationQuaternion TorsoControlJump::decomposeRotation(
    const RotationQuaternion& AqB, const Vector& vB) {

  const Vector vA = AqB.inverseRotate(vB).normalized();

  Vector rotAxis = (vA.cross(vB).normalized());

  if (rotAxis.norm() == 0) {
    rotAxis = Vector::UnitZ();
  }
  rotAxis *= -1.0;
  double rotAngle = -safeACOS(vA.dot(vB));
  const RotationQuaternion TqA = RotationQuaternion(
      AngleAxis(rotAngle, rotAxis.toImplementation()));
  return AqB * TqA;  // TqB
}

RotationQuaternion TorsoControlJump::computeHeading(
    const RotationQuaternion& rquat, const Vector& axis) {
  return decomposeRotation(rquat.conjugated(), axis).conjugated();

}

CoMOverSupportPolygonControl* TorsoControlJump::getCoMControl() {
  return &comControl_;
}

bool TorsoControlJump::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle hJump(handle.FirstChild("TorsoControl").FirstChild("Jump"));
  if (!comControl_.loadParameters(hJump)) {
    return false;
  }

//  std::cout << desiredTrajectory_.getInfoString() << std::endl;

return true;
}

} /* namespace loco */
