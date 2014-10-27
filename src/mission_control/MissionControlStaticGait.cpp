/*
 * MissionControlStaticGait.cpp
 *
 *  Created on: Aug 22, 2014
 *      Author: gech
 */

#include "loco/mission_control/MissionControlStaticGait.hpp"
#include "loco/mission_control/MissionControlSpeedFilter.hpp"

#include "loco/torso_control/TorsoControlStaticGait.hpp"
#include "loco/torso_control/TorsoControlDynamicGaitFreePlane.hpp"

#include "loco/foot_placement_strategy/FootPlacementStrategyStaticGait.hpp"


//colored strings
const std::string black     = "\033[0;30m";
const std::string red       = "\033[0;31m";
const std::string green     = "\033[0;32m";
const std::string yellow    = "\033[0;33m";
const std::string blue      = "\033[0;34m";
const std::string magenta   = "\033[0;35m";
const std::string cyan      = "\033[0;36m";
const std::string white     = "\033[0;37m";
const std::string def       = "\033[0m";


namespace loco {

MissionControlStaticGait::MissionControlStaticGait(robotModel::RobotModel* robotModel,   LocomotionControllerDynamicGait* locomotionController):
    robotModel_(robotModel),
    isExternallyVelocityControlled_(false),
    locomotionController_(locomotionController),
    speedFilter_(),
    useRosService_(false)
{

}

MissionControlStaticGait::~MissionControlStaticGait() {

}

bool MissionControlStaticGait::initialize(double dt) {
  isExternallyVelocityControlled_ = false;
  loco::FootPlacementStrategyStaticGait* fps = static_cast<loco::FootPlacementStrategyStaticGait*>(locomotionController_->getFootPlacementStrategy());
  useRosService_ = fps->isUsingRosService();

  std::cout << magenta << "[MissionController/init] "
            << blue << "ROS foothold service is: "
            << red << ( useRosService_ ? std::string{"enabled"} : std::string{"disabled"} )
            << def << std::endl;

  return true;
}

bool MissionControlStaticGait::advance(double dt) {

  const double minStrideDuration_ = 0.6;
  const double maxStrideDuration_ = 0.9;
  const double stepStrideDuration_ = 0.05;

  robotUtils::Joystick* joyStick = robotModel_->sensors().getJoystick();

  Twist desiredBaseTwistInHeadingFrame;
  Position desiredPositionMiddleOfFeetToBaseInWorldFrame;
  RotationQuaternion orientationOffset;

  bool standing = true;
  for (auto leg: *locomotionController_->getLegs()) {
    standing &= leg->isInStandConfiguration();
  }

  if (standing) {
    desiredPositionMiddleOfFeetToBaseInWorldFrame.z() = interpolateJoystickAxis(joyStick->getVertical(), speedFilter_.getMinimalPositionOffsetInWorldFrame().z(), speedFilter_.getMaximalPositionOffsetInWorldFrame().z());
    speedFilter_.setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(desiredPositionMiddleOfFeetToBaseInWorldFrame);

    orientationOffset = EulerAnglesZyx(0.0, M_PI/4, 0.0);

//    printf("sag: %lf \t cor: %lf  \t yaw: %lf\n", joyStick->getSagittal(), joyStick->getCoronal(), joyStick->getYaw());
    EulerAnglesZyx desEulerAnglesZyx;
    desEulerAnglesZyx.setRoll(interpolateJoystickAxis(-joyStick->getCoronal(), speedFilter_.getMinimalDesiredOrientationOffset().getUnique().roll(), speedFilter_.getMaximalDesiredOrientationOffset().getUnique().roll()));
    desEulerAnglesZyx.setPitch(interpolateJoystickAxis(joyStick->getSagittal(), speedFilter_.getMinimalDesiredOrientationOffset().getUnique().pitch(), speedFilter_.getMaximalDesiredOrientationOffset().getUnique().pitch()));
    desEulerAnglesZyx.setYaw(interpolateJoystickAxis(joyStick->getYaw(), speedFilter_.getMinimalDesiredOrientationOffset().getUnique().yaw(), speedFilter_.getMaximalDesiredOrientationOffset().getUnique().yaw()));
    orientationOffset(desEulerAnglesZyx.getUnique());
    speedFilter_.setUnfilteredDesiredOrientationOffset(orientationOffset);

    desiredBaseTwistInHeadingFrame.setZero();
  }
  else {
    desiredBaseTwistInHeadingFrame.getTranslationalVelocity().x() = joyStick->getSagittal();
    desiredBaseTwistInHeadingFrame.getTranslationalVelocity().y() = joyStick->getCoronal();
    desiredBaseTwistInHeadingFrame.getTranslationalVelocity().z() = 0.0;
    desiredBaseTwistInHeadingFrame.getRotationalVelocity().z() = joyStick->getYaw();
    speedFilter_.setUnfilteredDesiredOrientationOffset(RotationQuaternion());
    speedFilter_.setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(Position());
  }


  if (joyStick->getButtonOneClick(1)) {
    std::cout << magenta << "[MissionController/advance] "
              << blue << "Going to " << red << "stand" << blue << " configuration."
              << def << std::endl;
    locomotionController_->getFootPlacementStrategy()->goToStand();
    loco::TorsoControlStaticGait& torsoController = static_cast<loco::TorsoControlStaticGait&>(locomotionController_->getTorsoController());
    torsoController.setIsInStandConfiguration(true);
  }

  if (joyStick->getButtonOneClick(2)) {
    std::cout << magenta << "[MissionController/advance] "
              << blue << "Going to " << red << "walk" << blue << " configuration."
              << def << std::endl;
    locomotionController_->getFootPlacementStrategy()->resumeWalking();
    loco::TorsoControlStaticGait& torsoController = static_cast<loco::TorsoControlStaticGait&>(locomotionController_->getTorsoController());
    torsoController.setIsInStandConfiguration(false);
  }

  if (joyStick->getButtonOneClick(3)) {
    loco::FootPlacementStrategyStaticGait* fps = static_cast<loco::FootPlacementStrategyStaticGait*>(locomotionController_->getFootPlacementStrategy());
    useRosService_ = !useRosService_;
    fps->setUseRosService(useRosService_);

    std::cout << magenta << "[MissionController/advance] "
              << blue << "ROS foothold service is now: "
              << red << ( fps->isUsingRosService() ? std::string{"enabled"} : std::string{"disabled"} )
              << def << std::endl;
  }

  if (joyStick->getButtonOneClick(4)) {
    isExternallyVelocityControlled_ = !isExternallyVelocityControlled_;
    if (isExternallyVelocityControlled_) {
      printf("Velocity is controlled from extern.\n");
    } else {
      printf("Velocity is controlled by joystick.\n");
    }
  }

  if (isExternallyVelocityControlled_) {
    desiredBaseTwistInHeadingFrame.getTranslationalVelocity().x() = robotModel_->sensors().getDesRobotVelocity()->getDesSagittalVelocity();
    desiredBaseTwistInHeadingFrame.getTranslationalVelocity().y() = robotModel_->sensors().getDesRobotVelocity()->getDesCoronalVelocity();
    desiredBaseTwistInHeadingFrame.getRotationalVelocity().z() = robotModel_->sensors().getDesRobotVelocity()->getDesTurningRate();

  }

  speedFilter_.setUnfilteredDesiredBaseTwistInHeadingFrame(desiredBaseTwistInHeadingFrame);
  speedFilter_.advance(dt);
//  if (standing) {
   Twist desiredTwist = speedFilter_.getDesiredBaseTwistInHeadingFrame();
//  }

  loco::LinearVelocity desLinearVelocityBaseInControlFrame = desiredTwist.getTranslationalVelocity();
  loco::LocalAngularVelocity desAngularVelocityBaseInControlFrame = desiredTwist.getRotationalVelocity();

//  std::cout << "speed: " <<desLinearVelocityBaseInControlFrame << std::endl;

  locomotionController_->getTorso()->getDesiredState().setLinearVelocityBaseInControlFrame(desLinearVelocityBaseInControlFrame);
  locomotionController_->getTorso()->getDesiredState().setAngularVelocityBaseInControlFrame(desAngularVelocityBaseInControlFrame);

//  std::cout << "pos offset:" << speedFilter_.getDesiredPositionOffsetInWorldFrame() << std::endl;

  loco::TorsoControlStaticGait& torsoController = (loco::TorsoControlStaticGait&)locomotionController_->getTorsoController();
  torsoController.setDesiredPositionOffetInWorldFrame(speedFilter_.getDesiredPositionOffsetInWorldFrame());
  torsoController.setDesiredOrientationOffset(speedFilter_.getDesiredOrientationOffset());

  return true;
}


bool MissionControlStaticGait::loadParameters(const TiXmlHandle& handle) {
  return speedFilter_.loadParameters(handle);
}

const Twist& MissionControlStaticGait::getDesiredBaseTwistInHeadingFrame() const {
//  return gaitSwitcher_->getLocomotionController()->getDesiredBaseTwistInHeadingFrame();
}

double MissionControlStaticGait::interpolateJoystickAxis(double value, double minValue, double maxValue) {
  return 0.5*(maxValue-minValue)*(value+1.0) + minValue;
}
} /* namespace loco */
