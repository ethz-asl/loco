/*
 * BaseControlDynamicGait.cpp
 *
 *  Created on: Feb 24, 2014
 *      Author: gech
 */

#include "loco/torso_control/TorsoControlDynamicGait.hpp"
#include "loco/temp_helpers/math.hpp"

#include <exception>
namespace loco {

TorsoControlDynamicGait::TorsoControlDynamicGait(LegGroup* legs, TorsoBase* torso,  TerrainModelBase* terrain):
  TorsoControlGaitContainer(legs, torso, terrain)
{
    comControl_ = new CoMOverSupportPolygonControlDynamicGait(legs);
}

TorsoControlDynamicGait::~TorsoControlDynamicGait() {

}


bool TorsoControlDynamicGait::initialize(double dt) {
  const Position foreHipPosition = legs_->getLeg(0)->getPositionWorldToHipInBaseFrame();
  const Position hindHipPosition = legs_->getLeg(2)->getPositionWorldToHipInBaseFrame();
  headingDistanceFromForeToHindInBaseFrame_ = foreHipPosition.x()-hindHipPosition.x();
//  std::cout << "head dist: " << headingDistanceFromForeToHindInBaseFrame_ << std::endl;

  return true;
}


CoMOverSupportPolygonControlDynamicGait* TorsoControlDynamicGait::getCoMOverSupportPolygonControl() {
  return comControl_;
}


bool TorsoControlDynamicGait::advance(double dt) {
  comControl_->advance(dt);

  const RotationQuaternion orientationWorldToHeading = torso_->getMeasuredState().getOrientationWorldToControl();

  Position lateralAndHeadingPositionInWorldFrame = comControl_->getPositionWorldToDesiredCoMInWorldFrame();

  const double desiredForeHeightAboveGroundInWorldFrame = desiredTorsoForeHeightAboveGroundInWorldFrameOffset_+desiredTorsoForeHeightAboveGroundInWorldFrame_.evaluate(torso_->getStridePhase());
  const double desiredHindHeightAboveGroundInWorldFrame = desiredTorsoHindHeightAboveGroundInWorldFrameOffset_+desiredTorsoHindHeightAboveGroundInWorldFrame_.evaluate(torso_->getStridePhase());
  const double desiredMiddleHeightAboveGroundInWorldFrame = (desiredForeHeightAboveGroundInWorldFrame + desiredHindHeightAboveGroundInWorldFrame)/2.0;
  Position desiredLateralAndHeadingPositionInWorldFrame = lateralAndHeadingPositionInWorldFrame;
  Position groundHeightInWorldFrame = desiredLateralAndHeadingPositionInWorldFrame;
  terrain_->getHeight(groundHeightInWorldFrame);
  Position positionWorldToDesiredBaseInWorldFrame(desiredLateralAndHeadingPositionInWorldFrame.x(), desiredLateralAndHeadingPositionInWorldFrame.y(), desiredMiddleHeightAboveGroundInWorldFrame+groundHeightInWorldFrame.z());
  positionWorldToDesiredBaseInWorldFrame += desiredPositionOffsetInWorldFrame_;

//  Position desiredTorsoPositionInWorldFrame(0.0, desiredLateralAndHeadingPositionInWorldFrame.y(), desiredMiddleHeightAboveGroundInWorldFrame+groundHeightInWorldFrame.z());

  /* --- desired orientation --- */

  // pitch angle
  double height = desiredHindHeightAboveGroundInWorldFrame-desiredForeHeightAboveGroundInWorldFrame;
  double pitchAngle = atan2(height,headingDistanceFromForeToHindInBaseFrame_);
  RotationQuaternion orientationControlDesiredHeadingToBase = RotationQuaternion(AngleAxis(pitchAngle, 0.0, 1.0, 0.0));

  const Position positionForeFeetMidPointInWorldFrame = (legs_->getLeftForeLeg()->getPositionWorldToFootInWorldFrame() + legs_->getRightForeLeg()->getPositionWorldToFootInWorldFrame())/0.5;
  const Position positionHindFeetMidPointInWorldFrame = (legs_->getLeftHindLeg()->getPositionWorldToFootInWorldFrame() + legs_->getRightHindLeg()->getPositionWorldToFootInWorldFrame())/0.5;

  Position positionError = comControl_->getPositionWorldToDesiredCoMInWorldFrame() - torso_->getMeasuredState().getPositionWorldToControlInWorldFrame();

  Position positionWorldToDesiredForeFeetMidPointInWorldFrame = positionForeFeetMidPointInWorldFrame+ positionError;
  Position positionWorldToDesiredHindFeetMidPointInWorldFrame = positionHindFeetMidPointInWorldFrame+ positionError;

  Vector desiredHeadingDirectionInWorldFrame = Vector(positionWorldToDesiredForeFeetMidPointInWorldFrame-positionWorldToDesiredHindFeetMidPointInWorldFrame);
  desiredHeadingDirectionInWorldFrame.z() = 0.0;

  const Position positionForeHipsMidPointInWorldFrame = (legs_->getLeftForeLeg()->getPositionWorldToHipInWorldFrame() + legs_->getRightForeLeg()->getPositionWorldToHipInWorldFrame())/0.5;
  const Position positionHindHipsMidPointInWorldFrame = (legs_->getLeftHindLeg()->getPositionWorldToHipInWorldFrame() + legs_->getRightHindLeg()->getPositionWorldToHipInWorldFrame())/0.5;


  Vector currentHeadingDirectionInWorldFrame = Vector(positionForeHipsMidPointInWorldFrame-positionHindHipsMidPointInWorldFrame);
  currentHeadingDirectionInWorldFrame.z() = 0.0;

  RotationQuaternion orientationHeadingToDesiredHeading;
  try {
    orientationHeadingToDesiredHeading.setFromVectors(currentHeadingDirectionInWorldFrame.toImplementation(),desiredHeadingDirectionInWorldFrame.toImplementation());
  } catch (std::exception& e)
  {
    std::cout << e.what() << '\n';
    std::cout << "currentHeadingDirectionInWorldFrame: " << currentHeadingDirectionInWorldFrame <<std::endl;
    std::cout << "desiredHeadingDirectionInWorldFrame: " << desiredHeadingDirectionInWorldFrame <<std::endl;
    orientationHeadingToDesiredHeading.setIdentity();
  }

  RotationQuaternion desOrientationWorldToBase = orientationControlDesiredHeadingToBase*desiredOrientationOffset_*orientationHeadingToDesiredHeading*orientationWorldToHeading;
  RotationQuaternion orientationControlToDesiredBase;
  /* --- end desired orientation --- */


  torso_->getDesiredState().setPositionControlToBaseInControlFrame(positionWorldToDesiredBaseInWorldFrame);
  torso_->getDesiredState().setOrientationControlToBase(orientationControlToDesiredBase);

//  torso_->getDesiredState().setPositionWorldToBaseInWorldFrame(positionWorldToDesiredBaseInWorldFrame);
//  torso_->getDesiredState().setOrientationWorldToBase(desOrientationWorldToBase);

//  torso_->getDesiredState().setBaseTwistInBaseFrame(Twist(desiredLinearVelocity, desiredAngularVelocity));


  /* if a stance leg lost contact, lower it to re-gain contact */
  for (auto leg : *legs_) {
//    if (leg->isInStanceMode()) {
    if (leg->isSupportLeg()) {
      Position positionWorldToFootInWorldFrame =  leg->getPositionWorldToFootInWorldFrame();

      if (!leg->isGrounded()) {
        positionWorldToFootInWorldFrame.z() -= 0.01;
      }
      const Position positionWorldToBaseInWorldFrame = torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
      const Position positionBaseToFootInWorldFrame = positionWorldToFootInWorldFrame - positionWorldToBaseInWorldFrame;
      const Position positionBaseToFootInBaseFrame = torso_->getMeasuredState().getOrientationWorldToBase().rotate(positionBaseToFootInWorldFrame);
      leg->setDesiredJointPositions(leg->getJointPositionsFromPositionBaseToFootInBaseFrame(positionBaseToFootInBaseFrame));
    }
  }
  return true;
}


CoMOverSupportPolygonControlDynamicGait* TorsoControlDynamicGait::getCoMControl() {
  return comControl_;
}


const CoMOverSupportPolygonControlDynamicGait& TorsoControlDynamicGait::getCoMControl() const {
  return *comControl_;
}


bool TorsoControlDynamicGait::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle hDynGait(handle.FirstChild("TorsoControl").FirstChild("DynamicGait"));
  if (!comControl_->loadParameters(hDynGait)) {
    return false;
  }
  if (!loadParametersHipConfiguration(hDynGait)) {
    return false;
  }

  return true;
}


bool TorsoControlDynamicGait::setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t) {
  const TorsoControlDynamicGait& controller1 = static_cast<const TorsoControlDynamicGait&>(torsoController1);
  const TorsoControlDynamicGait& controller2 = static_cast<const TorsoControlDynamicGait&>(torsoController2);
  this->comControl_->setToInterpolated(controller1.getCoMControl(), controller2.getCoMControl(), t);
  desiredTorsoForeHeightAboveGroundInWorldFrameOffset_ = linearlyInterpolate(controller1.getDesiredTorsoForeHeightAboveGroundInWorldFrameOffset(),
                                                                             controller2.getDesiredTorsoForeHeightAboveGroundInWorldFrameOffset(),
                                                                             0.0,
                                                                             1.0,
                                                                             t);
  desiredTorsoHindHeightAboveGroundInWorldFrameOffset_ = linearlyInterpolate(controller1.getDesiredTorsoHindHeightAboveGroundInWorldFrameOffset(),
                                                                             controller2.getDesiredTorsoHindHeightAboveGroundInWorldFrameOffset(),
                                                                             0.0,
                                                                             1.0,
                                                                             t);


  if(!interpolateHeightTrajectory(desiredTorsoForeHeightAboveGroundInWorldFrame_, controller1.desiredTorsoForeHeightAboveGroundInWorldFrame_, controller2.desiredTorsoForeHeightAboveGroundInWorldFrame_, t)) {
    return false;
  }
  if(!interpolateHeightTrajectory(desiredTorsoHindHeightAboveGroundInWorldFrame_, controller1.desiredTorsoHindHeightAboveGroundInWorldFrame_, controller2.desiredTorsoHindHeightAboveGroundInWorldFrame_, t)) {
    return false;
  }

  return true;
}


} /* namespace loco */
