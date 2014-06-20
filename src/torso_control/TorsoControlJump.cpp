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

  std::vector<double> tValues, xValues;
  const double defaultHeight = 0.42;
  desiredTorsoForeHeightAboveGroundInWorldFrameOffset_ = defaultHeight;
  desiredTorsoHindHeightAboveGroundInWorldFrameOffset_ = defaultHeight;
  tValues.push_back(0.00);
  xValues.push_back(0.0);
  tValues.push_back(0.25);
  xValues.push_back(0.0);
  tValues.push_back(0.50);
  xValues.push_back(0.0);
  tValues.push_back(0.75);
  xValues.push_back(0.0);
  tValues.push_back(1.00);
  xValues.push_back(0.0);
  desiredTorsoForeHeightAboveGroundInWorldFrame_.setRBFData(tValues, xValues);
  desiredTorsoHindHeightAboveGroundInWorldFrame_.setRBFData(tValues, xValues);
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
  const double desiredForeHeightAboveGroundInWorldFrame =
      desiredTorsoForeHeightAboveGroundInWorldFrameOffset_
          + desiredTorsoForeHeightAboveGroundInWorldFrame_.evaluate(
              torso_->getStridePhase());
  const double desiredHindHeightAboveGroundInWorldFrame =
      desiredTorsoHindHeightAboveGroundInWorldFrameOffset_
          + desiredTorsoHindHeightAboveGroundInWorldFrame_.evaluate(
              torso_->getStridePhase());
  const double desiredMiddleHeightAboveGroundInWorldFrame =
      (desiredForeHeightAboveGroundInWorldFrame
          + desiredHindHeightAboveGroundInWorldFrame) / 2.0;

  Position desiredLateralAndHeadingPositionInWorldFrame =
      lateralAndHeadingPositionInWorldFrame;
  Position groundHeightInWorldFrame =
      desiredLateralAndHeadingPositionInWorldFrame;
  terrain_->getHeight(groundHeightInWorldFrame);
  Position desiredTorsoPositionInWorldFrame(
      desiredLateralAndHeadingPositionInWorldFrame.x(),
      desiredLateralAndHeadingPositionInWorldFrame.y(),
      desiredMiddleHeightAboveGroundInWorldFrame
          + groundHeightInWorldFrame.z());
//  Position desiredTorsoPositionInWorldFrame(0.0, desiredLateralAndHeadingPositionInWorldFrame.y(), desiredMiddleHeightAboveGroundInWorldFrame+groundHeightInWorldFrame.z());

  /* --- desired orientation --- */

  // pitch angle
  double height = desiredHindHeightAboveGroundInWorldFrame
      - desiredForeHeightAboveGroundInWorldFrame;
  double pitchAngle = atan2(height, headingDistanceFromForeToHindInBaseFrame_);
  RotationQuaternion orientationDesiredHeadingToBase = RotationQuaternion(
      AngleAxis(pitchAngle, 0.0, 1.0, 0.0));

  const Position positionForeFeetMidPointInWorldFrame = (legs_->getLeftForeLeg()
      ->getWorldToFootPositionInWorldFrame()
      + legs_->getRightForeLeg()->getWorldToFootPositionInWorldFrame()) / 0.5;
  const Position positionHindFeetMidPointInWorldFrame = (legs_->getLeftHindLeg()
      ->getWorldToFootPositionInWorldFrame()
      + legs_->getRightHindLeg()->getWorldToFootPositionInWorldFrame()) / 0.5;
  Position positionWorldToDesiredForeFeetMidPointInWorldFrame =
      positionForeFeetMidPointInWorldFrame
          + comControl_.getPositionErrorVectorInWorldFrame();
  Position positionWorldToDesiredHindFeetMidPointInWorldFrame =
      positionHindFeetMidPointInWorldFrame
          + comControl_.getPositionErrorVectorInWorldFrame();

  Vector desiredHeadingDirectionInWorldFrame = Vector(
      positionWorldToDesiredForeFeetMidPointInWorldFrame
          - positionWorldToDesiredHindFeetMidPointInWorldFrame);
  desiredHeadingDirectionInWorldFrame.z() = 0.0;

  const Position positionForeHipsMidPointInWorldFrame = (legs_->getLeftForeLeg()
      ->getWorldToHipPositionInWorldFrame()
      + legs_->getRightForeLeg()->getWorldToHipPositionInWorldFrame()) / 0.5;
  const Position positionHindHipsMidPointInWorldFrame = (legs_->getLeftHindLeg()
      ->getWorldToHipPositionInWorldFrame()
      + legs_->getRightHindLeg()->getWorldToHipPositionInWorldFrame()) / 0.5;

  Vector currentHeadingDirectionInWorldFrame = Vector(
      positionForeHipsMidPointInWorldFrame
          - positionHindHipsMidPointInWorldFrame);
  currentHeadingDirectionInWorldFrame.z() = 0.0;

  RotationQuaternion orientationHeadingToDesiredHeading;
  try {
    orientationHeadingToDesiredHeading.setFromVectors(
        currentHeadingDirectionInWorldFrame.toImplementation(),
        desiredHeadingDirectionInWorldFrame.toImplementation());
  } catch (std::exception& e) {
    std::cout << e.what() << '\n';
    std::cout << "currentHeadingDirectionInWorldFrame: "
        << currentHeadingDirectionInWorldFrame << std::endl;
    std::cout << "desiredHeadingDirectionInWorldFrame: "
        << desiredHeadingDirectionInWorldFrame << std::endl;
    orientationHeadingToDesiredHeading.setIdentity();
  }

  RotationQuaternion desOrientationWorldToBase = orientationDesiredHeadingToBase
      * orientationHeadingToDesiredHeading * orientationWorldToHeading;

  /* --- end desired orientation --- */

  torso_->getDesiredState().setWorldToBasePoseInWorldFrame(
      Pose(desiredTorsoPositionInWorldFrame, desOrientationWorldToBase));
//  torso_->getDesiredState().setBaseTwistInBaseFrame(Twist(desiredLinearVelocity, desiredAngularVelocity));

  /* if a stance leg lost contact, lower it to re-gain contact */
  /*for (auto leg : *legs_) {
   if (leg->isInStanceMode()) {
   Position positionWorldToFootInWorldFrame =  leg->getWorldToFootPositionInWorldFrame();


   if (!leg->isGrounded()) {
   positionWorldToFootInWorldFrame.z() -= 0.01;
   }
   const Position positionWorldToBaseInWorldFrame = torso_->getMeasuredState().getWorldToBasePositionInWorldFrame();
   const Position positionBaseToFootInWorldFrame = positionWorldToFootInWorldFrame - positionWorldToBaseInWorldFrame;
   const Position positionBaseToFootInBaseFrame = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame().rotate(positionBaseToFootInWorldFrame);
   leg->setDesiredJointPositions(leg->getJointPositionsFromBaseToFootPositionInBaseFrame(positionBaseToFootInBaseFrame));
   }
   }*/
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
  if (!loadParametersHipConfiguration(hJump)) {
    return false;
  }
  if (!loadTrajectory(hJump, desiredTrajectory_)) {
    return false;
  }

  std::cout << desiredTrajectory_.getInfoString() << std::endl;

  return true;
}

bool TorsoControlJump::loadParametersHipConfiguration(
    const TiXmlHandle &hParameterSet) {
  int iKnot;
  double t, value;
  TiXmlElement* pElem;
  std::string legFrame;

  /* Swing foot configuration*/
  pElem = hParameterSet.FirstChild("HipConfiguration").Element();
  if (!pElem) {
    printf("Could not find HipConfiguration\n");
    return false;
  }

  /**************************************************************************
   * HEIGHT
   ***************************************************************************/

  /* offset */
  pElem = hParameterSet.FirstChild("HipConfiguration").Element();
  if (!pElem) {
    printf("Could not find HipConfiguration!\n");
    return false;
  }
  TiXmlElement* child =
      hParameterSet.FirstChild("HipConfiguration").FirstChild().ToElement();
  for (child; child; child = child->NextSiblingElement()) {
    if (child->ValueStr().compare("HeightTrajectory") == 0) {
      bool isFore = false;
      bool isHind = false;
      double offset = 0.0;
      if (child->QueryDoubleAttribute("offset", &offset) != TIXML_SUCCESS) {
        printf("Could not find offset!\n");
      }
      if (child->QueryBoolAttribute("fore", &isFore) == TIXML_SUCCESS) {
        if (isFore) {
          desiredTorsoForeHeightAboveGroundInWorldFrameOffset_ = offset;
          TiXmlHandle hTrajectory(child);
          if (!loadHeightTrajectory(
              hTrajectory, desiredTorsoForeHeightAboveGroundInWorldFrame_)) {
            return false;
          }
        }
      }
      if (child->QueryBoolAttribute("hind", &isHind) == TIXML_SUCCESS) {
        if (isHind) {
          desiredTorsoHindHeightAboveGroundInWorldFrameOffset_ = offset;
          TiXmlHandle hTrajectory(child);
          if (!loadHeightTrajectory(
              hTrajectory, desiredTorsoHindHeightAboveGroundInWorldFrame_)) {
            return false;
          }
        }
      }
    }

  }

  return true;
}

bool TorsoControlJump::loadHeightTrajectory(const TiXmlHandle &hTrajectory,
                                            rbf::PeriodicRBF1DC1& trajectory) {
  TiXmlElement* pElem;
  int iKnot;
  double t, value;
  std::vector<double> tValues, xValues;

  TiXmlElement* child = hTrajectory.FirstChild().ToElement();
  for (child; child; child = child->NextSiblingElement()) {
    if (child->QueryDoubleAttribute("t", &t) != TIXML_SUCCESS) {
      printf("Could not find t of knot!\n");
      return false;
    }
    if (child->QueryDoubleAttribute("v", &value) != TIXML_SUCCESS) {
      printf("Could not find v of knot!\n");
      return false;
    }
    tValues.push_back(t);
    xValues.push_back(value);
  }
  trajectory.setRBFData(tValues, xValues);

  return true;
}

bool TorsoControlJump::loadTrajectory(const TiXmlHandle &hTrajectory,
                                      dmp::GaussianKernel& trajectory) {
  TiXmlElement* pElem;
  int numBasisFunctions;
  double activation, canSysCutOff;
  bool exponentiallySpaced;

  pElem = hTrajectory.FirstChild("Trajectory").FirstChild("GaussianKernel")
      .Element();
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

  trajectory.initialize(numBasisFunctions, activation, exponentiallySpaced,
                        canSysCutOff);
  return true;
}

} /* namespace loco */
