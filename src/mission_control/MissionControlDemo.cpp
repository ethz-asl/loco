/*
 * MissionControlDemo.cpp
 *
 *  Created on: Aug 22, 2014
 *      Author: gech
 */

#include "loco/mission_control/MissionControlDemo.hpp"
#include "loco/mission_control/MissionControlSpeedFilter.hpp"

namespace loco {

MissionControlDemo::MissionControlDemo(robotModel::RobotModel* robotModel,   GaitSwitcherDynamicGaitDefault* gaitSwitcher):
    robotModel_(robotModel),
    gaitSwitcher_(gaitSwitcher),
    isExternallyVelocityControlled_(false)
{

}

MissionControlDemo::~MissionControlDemo() {

}

bool MissionControlDemo::initialize(double dt) {
  isExternallyVelocityControlled_ = false;
  return true;
}

bool MissionControlDemo::advance(double dt) {

  const double minStrideDuration_ = 0.6;
  const double maxStrideDuration_ = 0.9;
  const double stepStrideDuration_ = 0.05;

  robotUtils::Joystick* joyStick = robotModel_->sensors().getJoystick();


  MissionControlSpeedFilter& missionController = gaitSwitcher_->getLocomotionController()->getMissionController();
  Twist desiredBaseTwistInHeadingFrame;

  if (gaitSwitcher_->getLocomotionController()->getGaitName() == "Stand") {
    Position desiredPositionMiddleOfFeetToBaseInWorldFrame;
    desiredPositionMiddleOfFeetToBaseInWorldFrame.z() = interpolateJoystickAxis(joyStick->getVertical(), missionController.getMinimalPositionOffsetInWorldFrame().z(), missionController.getMaximalPositionOffsetInWorldFrame().z());
    missionController.setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(desiredPositionMiddleOfFeetToBaseInWorldFrame);

    RotationQuaternion orientationOffset(EulerAnglesZyx(0.0, M_PI/4, 0.0));

//    printf("sag: %lf \t cor: %lf  \t yaw: %lf\n", joyStick->getSagittal(), joyStick->getCoronal(), joyStick->getYaw());
    EulerAnglesZyx desEulerAnglesZyx;
    desEulerAnglesZyx.setRoll(interpolateJoystickAxis(joyStick->getCoronal(), missionController.getMinimalDesiredOrientationOffset().getUnique().roll(), missionController.getMaximalDesiredOrientationOffset().getUnique().roll()));
    desEulerAnglesZyx.setPitch(interpolateJoystickAxis(joyStick->getSagittal(), missionController.getMinimalDesiredOrientationOffset().getUnique().pitch(), missionController.getMaximalDesiredOrientationOffset().getUnique().pitch()));
    desEulerAnglesZyx.setYaw(interpolateJoystickAxis(joyStick->getYaw(), missionController.getMinimalDesiredOrientationOffset().getUnique().yaw(), missionController.getMaximalDesiredOrientationOffset().getUnique().yaw()));
    orientationOffset(desEulerAnglesZyx.getUnique());
    missionController.setUnfilteredDesiredOrientationOffset(orientationOffset);


    desiredBaseTwistInHeadingFrame.setZero();
  } else if (gaitSwitcher_->getLocomotionController()->getGaitName() == "WalkingTrot") {


    // xbox cross down
    if (joyStick->getButtonOneClick(15)) { // 15
      double strideDuration = gaitSwitcher_->getLocomotionController()->getGaitPattern()->getStrideDuration() + stepStrideDuration_;
      strideDuration = boundToRange(strideDuration, minStrideDuration_,  maxStrideDuration_);
      gaitSwitcher_->getLocomotionController()->getGaitPattern()->setStrideDuration(strideDuration);
      printf("Increased stride duration t=%f!\n", strideDuration);
    }

    // xbox cross upo
    if (joyStick->getButtonOneClick(14)) { // 14
      double strideDuration = gaitSwitcher_->getLocomotionController()->getGaitPattern()->getStrideDuration() - stepStrideDuration_;
      strideDuration = boundToRange(strideDuration, minStrideDuration_,  maxStrideDuration_);
      gaitSwitcher_->getLocomotionController()->getGaitPattern()->setStrideDuration(strideDuration);
      printf("Decreased stride duration t=%f!\n", strideDuration);
    }

  }else {

    desiredBaseTwistInHeadingFrame.getTranslationalVelocity().x() = joyStick->getSagittal();
    desiredBaseTwistInHeadingFrame.getTranslationalVelocity().y() = joyStick->getCoronal();
    desiredBaseTwistInHeadingFrame.getTranslationalVelocity().z() = 0.0;
    desiredBaseTwistInHeadingFrame.getRotationalVelocity().z() = joyStick->getYaw();

    missionController.setUnfilteredDesiredOrientationOffset(RotationQuaternion());
    missionController.setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(Position());

  }



  if (joyStick->getButtonOneClick(1)) {
    gaitSwitcher_->transitToGait("Stand");
  }

  if (joyStick->getButtonOneClick(2)) {
    gaitSwitcher_->transitToGait("StaticLateralWalk");
  }

  if (joyStick->getButtonOneClick(3)) {
    gaitSwitcher_->transitToGait("WalkingTrot");
  }

  if (joyStick->getButtonOneClick(4)) {
    isExternallyVelocityControlled_ = !isExternallyVelocityControlled_;
  }

  if (isExternallyVelocityControlled_) {
    desiredBaseTwistInHeadingFrame.getTranslationalVelocity().x() = robotModel_->sensors().getDesRobotVelocity()->getDesSagittalVelocity();
    desiredBaseTwistInHeadingFrame.getTranslationalVelocity().y() = robotModel_->sensors().getDesRobotVelocity()->getDesCoronalVelocity();
    desiredBaseTwistInHeadingFrame.getRotationalVelocity().z() = robotModel_->sensors().getDesRobotVelocity()->getDesTurningRate();
  }

  gaitSwitcher_->getLocomotionController()->setDesiredBaseTwistInHeadingFrame(desiredBaseTwistInHeadingFrame);

  return true;
}


bool MissionControlDemo::loadParameters(const TiXmlHandle& handle) {
  return true;
}

const Twist& MissionControlDemo::getDesiredBaseTwistInHeadingFrame() const {
  return gaitSwitcher_->getLocomotionController()->getDesiredBaseTwistInHeadingFrame();
}

double MissionControlDemo::interpolateJoystickAxis(double value, double minValue, double maxValue) {
  return 0.5*(maxValue-minValue)*(value+1.0) + minValue;
}
} /* namespace loco */
