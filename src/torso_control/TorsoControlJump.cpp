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
      headingDistanceFromForeToHindInBaseFrame_(0.0) {
  output_.open("./output.txt");
  currentTime_ = 0.0;
}

TorsoControlJump::~TorsoControlJump() {
  output_.close();
  delete thetas_;
}

bool TorsoControlJump::initialize(double dt) {
  const Position foreHipPosition = legs_->getLeg(0)
      ->getWorldToHipPositionInBaseFrame();
  const Position hindHipPosition = legs_->getLeg(2)
      ->getWorldToHipPositionInBaseFrame();
  headingDistanceFromForeToHindInBaseFrame_ = foreHipPosition.x()
      - hindHipPosition.x();

  return true;
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

  incrementTime(dt);
  desiredTrajectory_.predict(getProgress(),
                             desiredTorsoHeightAboveGroundInWorldFrame, false);

  Position desiredTorsoPositionInWorldFrame(
      desiredLateralAndHeadingPositionInWorldFrame.x(),
      desiredLateralAndHeadingPositionInWorldFrame.y(),
      desiredTorsoHeightAboveGroundInWorldFrame + groundHeightInWorldFrame.z());

  //std::cout << desiredTorsoHeightAboveGroundInWorldFrame << std::endl;

  /* --- desired orientation --- */

  // Just keep torso parallel to ground for now
  RotationQuaternion desOrientationWorldToBase = RotationQuaternion();

  /* --- end desired orientation --- */

  torso_->getDesiredState().setWorldToBasePoseInWorldFrame(
      Pose(desiredTorsoPositionInWorldFrame, desOrientationWorldToBase));
  addMeasuresToTrajectory(
      torso_->getMeasuredState().getWorldToBasePositionInWorldFrame().z());
  output_ << currentTime_ << " "
          << torso_->getMeasuredState().getWorldToBasePositionInWorldFrame().z()
          << std::endl;
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

/**
 * Ensures that a number is clamped between a lower bound and an upper bound.
 * @param number: Number to be clamped.
 * @param lower: Lower bound.
 * @param upper: Upper bound.
 */
inline double clamp(double number, double lower, double upper) {
  if (number > upper) {
    number = upper;
  } else if (number < lower) {
    number = lower;
  }
  return number;
}

/**
 * Returns max duration of jump.
 */
double TorsoControlJump::getMaxDuration() {
  return maxDuration_;
}

/**
 * Resets the timer.
 */
void TorsoControlJump::resetTime() {
  currentTime_ = 0.0;
}

/**
 * Increments the timer with timestep dt.
 */
void TorsoControlJump::incrementTime(double dt) {
  currentTime_ += dt;
}

/**
 * Keeps track of how much of the trajectory has already been followed in terms of progress (between 0 and 1).
 */
double TorsoControlJump::getProgress() {
  return clamp(currentTime_ / maxDuration_, 0.0, 1.0);
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
  if (!loadTrajectory(hJump)) {
    return false;
  }

//  std::cout << desiredTrajectory_.getInfoString() << std::endl;

  return true;
}

/**
 * Loads parameters for the trajectory to follow.
 */
bool TorsoControlJump::loadTrajectory(const TiXmlHandle &hJump) {
  TiXmlHandle hTrajectory(hJump.FirstChild("Trajectory"));

  if (!loadGaussianKernel(hTrajectory)) {
    return false;
  }
  if (!loadMovement(hTrajectory)) {
    return false;
  }
  return true;
}

/**
 * Loads parameters for the GaussianKernel to shape the trajectory.
 * @param hTrajectory: Parent XML tag of the GaussianKernel tag.
 */
bool TorsoControlJump::loadGaussianKernel(const TiXmlHandle &hTrajectory) {
  TiXmlElement* pElem;
  int numBasisFunctions;
  double activation, canSysCutOff;
  bool exponentiallySpaced;

  pElem = hTrajectory.FirstChild("GaussianKernel").Element();
  if (!pElem) {

    printf("Could not find Jump:Trajectory:GaussianKernel\n");
    return false;
  }

  TiXmlElement* child = hTrajectory.FirstChild("GaussianKernel").FirstChild()
      .ToElement();

  TiXmlHandle hThetas(child);

  if (!loadThetas(hThetas)) {
    return false;
  }

  numBasisFunctions = thetas_->size();

  if (pElem->QueryDoubleAttribute("activation", &activation) != TIXML_SUCCESS) {
    printf("Could not find activation parameter!\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("canSysCutOff", &canSysCutOff)
      != TIXML_SUCCESS) {
    printf("Could not find parameter canSysCutOff!\n");
    return false;
  }

  if (pElem->QueryBoolAttribute("exponentiallySpaced", &exponentiallySpaced)
      != TIXML_SUCCESS) {
    printf("Could not find parameter exponentiallySpaced!\n");
    return false;
  }

  desiredTrajectory_.initialize(numBasisFunctions, activation,
                                exponentiallySpaced, canSysCutOff);

  /* Print out thetas to check correctness */
//  for (int i = 0; i < thetas_->size(); i++) {
//    std::cout << "theta[" << i << "] = " << (*thetas_)(i) << std::endl;
//  }
  desiredTrajectory_.setThetas(*thetas_);

  return true;
}

/**
 * Loads parameter for the duration of the jump.
 * @param hTrajectory: Parent XML tag of the Movement tag.
 */
bool TorsoControlJump::loadMovement(const TiXmlHandle &hTrajectory) {
  double maxDuration;

  TiXmlElement *pElem;
  pElem = hTrajectory.FirstChild("Movement").Element();
  if (!pElem) {
    printf("Could not find Jump:Trajectory:Movement\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("maxDuration", &maxDuration)
      != TIXML_SUCCESS) {
    printf("Could not find parameter maxDuration!\n");
    return false;
  }

  maxDuration_ = maxDuration;
  return true;
}

/**
 * Loads thetas for GaussianKernel.
 * @param hTrajectory: Parent XML tag of the Movement tag.
 */
bool TorsoControlJump::loadThetas(const TiXmlHandle &hThetas) {
  TiXmlElement* pElem;
  double value;
  std::vector<double> values;

  TiXmlElement* child = hThetas.FirstChild().ToElement();
  for (child; child; child = child->NextSiblingElement()) {
    if (child->QueryDoubleAttribute("v", &value) != TIXML_SUCCESS) {
      printf("Could not find value of theta!\n");
      return false;
    }
    values.push_back(value);
  }

  thetas_ = new Eigen::VectorXd(values.size());

  for (int i = 0; i < values.size(); i++) {
    (*thetas_)[i] = values.at(i);
  }

  return true;
}

} /* namespace loco */
