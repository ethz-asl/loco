/*
 * MissionControlJoystick.cpp
 *
 *  Created on: Mar 7, 2014
 *      Author: gech
 */

#include "loco/mission_control/MissionControlJoystick.hpp"
#include <limits>
#include "loco/temp_helpers/math.hpp"

namespace loco {

MissionControlJoystick::MissionControlJoystick(robotModel::RobotModel* robotModel)
: robotModel_(robotModel),
  baseTwistInHeadingFrame_(),
  maximumBaseTwistInHeadingFrame_(LinearVelocity(std::numeric_limits<LinearVelocity::Scalar>::max(), std::numeric_limits<LinearVelocity::Scalar>::max(), std::numeric_limits<LinearVelocity::Scalar>::max()), LocalAngularVelocity(std::numeric_limits<LinearVelocity::Scalar>::max(), std::numeric_limits<LinearVelocity::Scalar>::max(), std::numeric_limits<LinearVelocity::Scalar>::max()) )
{
  filteredVelocities_[0].setAlpha(0.005);
  filteredVelocities_[1].setAlpha(0.05);
  filteredVelocities_[2].setAlpha(0.05);
}

MissionControlJoystick::~MissionControlJoystick() {

}

bool MissionControlJoystick::initialize(double dt) {

  return true;
}

void MissionControlJoystick::advance(double dt) {
  robotUtils::Joystick* joyStick = robotModel_->sensors().getJoystick();
  const double maxHeadingVel = maximumBaseTwistInHeadingFrame_.getTranslationalVelocity().x();
  filteredVelocities_[0].update(joyStick->getSagittal());
  double headingVel = filteredVelocities_[0].val();
  boundToRange(&headingVel, -maxHeadingVel, maxHeadingVel);

  const double maxLateralVel = maximumBaseTwistInHeadingFrame_.getTranslationalVelocity().y();
  filteredVelocities_[1].update(joyStick->getCoronal());

  double lateralVel = filteredVelocities_[1].val();
  boundToRange(&lateralVel, -maxLateralVel, maxLateralVel);
  LinearVelocity linearVelocity(headingVel, lateralVel, 0.0);

  const double maxTurningVel = maximumBaseTwistInHeadingFrame_.getRotationalVelocity().z();
  filteredVelocities_[2].update(joyStick->getYaw());
  double turningVel = filteredVelocities_[2].val();
  boundToRange(&turningVel, -maxTurningVel, maxTurningVel);
  LocalAngularVelocity angularVelocity(0.0, 0.0, turningVel);

  baseTwistInHeadingFrame_ = Twist(linearVelocity, angularVelocity);
}

const Twist& MissionControlJoystick::getDesiredBaseTwistInHeadingFrame() const {
  return baseTwistInHeadingFrame_;
}

bool MissionControlJoystick::loadParameters(const TiXmlHandle& handle) {

  TiXmlElement* pElem;

  /* maximum */
  TiXmlHandle hMaxSpeed(handle.FirstChild("LocomotionController").FirstChild("Mission").FirstChild("Speed").FirstChild("Maximum"));
  pElem = hMaxSpeed.Element();
  if (!pElem) {
    printf("Could not find Mission:Speed:Maximum\n");
    return false;
  } else {
    if (pElem->QueryDoubleAttribute("headingSpeed", &maximumBaseTwistInHeadingFrame_.getTranslationalVelocity().x())!=TIXML_SUCCESS) {
      printf("Could not find Speed:Maximum:headingSpeed\n");
      return false;
    }
    if (pElem->QueryDoubleAttribute("lateralSpeed", &maximumBaseTwistInHeadingFrame_.getTranslationalVelocity().y())!=TIXML_SUCCESS) {
      printf("Could not find Speed:Maximum:lateralSpeed\n");
      return false;
    }


    if (pElem->QueryDoubleAttribute("turningSpeed", &maximumBaseTwistInHeadingFrame_.getRotationalVelocity().z())!=TIXML_SUCCESS) {
      printf("Could not find Speed:Maximum:turningSpeed\n");
      return false;
    }
  }
  return true;
}

} /* namespace loco */
