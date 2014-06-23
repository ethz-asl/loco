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

  const double defaultHeight = 0.42;
  desiredTorsoForeHeightAboveGroundInWorldFrameOffset_ = defaultHeight;
  desiredTorsoHindHeightAboveGroundInWorldFrameOffset_ = defaultHeight;
}

TorsoControlJump::~TorsoControlJump() {

}
bool TorsoControlJump::initialize(double dt) {
  const Position foreHipPosition = legs_->getLeg(0)
      ->getWorldToHipPositionInBaseFrame();
  const Position hindHipPosition = legs_->getLeg(2)
      ->getWorldToHipPositionInBaseFrame();
  headingDistanceFromForeToHindInBaseFrame_ = foreHipPosition.x()
      - hindHipPosition.x();
//  std::cout << "head dist: " << headingDistanceFromForeToHindInBaseFrame_ << std::endl;

  return true;
}

void TorsoControlJump::advance(double dt) {
  comControl_.advance(dt);

  const RotationQuaternion orientationWorldToHeading =
      torso_->getMeasuredState().getWorldToHeadingOrientation();

  //TODO: Check if anything needs to be changed here in comControl
  Position lateralAndHeadingPositionInWorldFrame = comControl_
      .getDesiredWorldToCoMPositionInWorldFrame();

  //TODO: Change these to make use of Jump trajectory instead of Periodic trajectory
  //TODO: Use predict to get a height over parameter x (goes from 0 to 1)

  Position desiredLateralAndHeadingPositionInWorldFrame =
      lateralAndHeadingPositionInWorldFrame;
  Position groundHeightInWorldFrame =
      desiredLateralAndHeadingPositionInWorldFrame;

  terrain_->getHeight(groundHeightInWorldFrame);

  /* Small test to see if torso follows a simple declining vertical trajectory */
  static double desiredTorsoHeightAboveGroundInWorldFrame = 1;
  if (desiredTorsoHeightAboveGroundInWorldFrame > 0) {
    desiredTorsoHeightAboveGroundInWorldFrame -= 0.001;
    std::cout << desiredTorsoHeightAboveGroundInWorldFrame << std::endl;
  }

  //TODO: Create Timer and introduce max_duration parameter to propagate Gaussian
//   double desiredTorsoHeightAboveGroundInWorldFrame;
//   desiredTrajectory_.predict(0.5, desiredTorsoHeightAboveGroundInWorldFrame,
//   false);

  Position desiredTorsoPositionInWorldFrame(
      0.0, desiredLateralAndHeadingPositionInWorldFrame.y(),
      desiredTorsoHeightAboveGroundInWorldFrame + groundHeightInWorldFrame.z());

//  std::cout << desiredTorsoHeightAboveGroundInWorldFrame << std::endl;

  /* --- desired orientation --- */

  // Just keep torso parallel to ground for now
  RotationQuaternion desOrientationWorldToBase = RotationQuaternion(0, 0, 0, 0);

  /* --- end desired orientation --- */

  torso_->getDesiredState().setWorldToBasePoseInWorldFrame(
      Pose(desiredTorsoPositionInWorldFrame, desOrientationWorldToBase));
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

  std::cout << desiredTrajectory_.getInfoString() << std::endl;

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

  if (pElem->QueryIntAttribute("numBasisFunctions", &numBasisFunctions)
      != TIXML_SUCCESS) {
    printf("Could not find parameter numBasisFunctions!\n");
    return false;
  }

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

  return true;
}

/**
 * Loads parameter for the duration of the jump.
 * @param hTrajectory: Parent XML tag of the Movement tag.
 */
bool TorsoControlJump::loadMovement(const TiXmlHandle &hTrajectory) {
  double maxDuration;cd

  TiXmlElement* pElem;
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

} /* namespace loco */
