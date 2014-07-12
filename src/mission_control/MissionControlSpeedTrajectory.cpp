/*
 * MissionControlZeroToConstantHeadingVelocity.cpp
 *
 *  Created on: Apr 4, 2014
 *      Author: gech
 */

#include "loco/mission_control/MissionControlSpeedTrajectory.hpp"

namespace loco {

MissionControlSpeedTrajectory::MissionControlSpeedTrajectory() :
  MissionControlBase(),
  time_(0.0),
  currentBaseTwistInHeadingFrame_(),
  isInterpolatingTime_(true),
  cycleDuration_(0.0)
{


}

MissionControlSpeedTrajectory::~MissionControlSpeedTrajectory() {

}

const Twist& MissionControlSpeedTrajectory::getDesiredBaseTwistInHeadingFrame() const {
  return currentBaseTwistInHeadingFrame_;
}

bool MissionControlSpeedTrajectory::initialize(double dt) {
  currentBaseTwistInHeadingFrame_.setZero();
  time_ = 0.0;
  return true;
}
bool MissionControlSpeedTrajectory::advance(double dt) {

  currentBaseTwistInHeadingFrame_.getTranslationalVelocity() = linearVelocityTrajectory_.evaluate_linear(time_);
  currentBaseTwistInHeadingFrame_.getRotationalVelocity() = localAngularVelocityTrajectory_.evaluate_linear(time_);
  time_ += dt;

  return true;

}
bool MissionControlSpeedTrajectory::loadParameters(const TiXmlHandle& handle) {
  linearVelocityTrajectory_.clear();
  localAngularVelocityTrajectory_.clear();

  TiXmlElement* pElem;
  double t, value;

  loco::TrajectoryLinearVelocity::Type linearVelocity;
  loco::TrajectoryLocalAngularVelocity::Type localAngularVelocity;

  TiXmlHandle hSpeedTrajectory(handle.FirstChild("LocomotionController").FirstChild("Mission").FirstChild("Speed").FirstChild("Trajectory"));
  pElem = hSpeedTrajectory.ToElement();
  if (!pElem) {
    printf("Could not find LocomotionController:Mission:Speed:Trajectory!\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("cycleDuration", &cycleDuration_)==TIXML_SUCCESS) {
	  isInterpolatingTime_ = false;
  } else {
	  isInterpolatingTime_ = true;
  }
  TiXmlElement* child = hSpeedTrajectory.FirstChild().ToElement();
   for( child; child; child=child->NextSiblingElement() ){
      if (child->QueryDoubleAttribute("t", &t)!=TIXML_SUCCESS) {
			printf("Could not find t  of knot!\n");
			return false;
      }
      if (child->QueryDoubleAttribute("headingSpeed", &linearVelocity.x())!=TIXML_SUCCESS) {
        printf("Could not find headingSpeed of knot!\n");
        return false;
      }
      if (child->QueryDoubleAttribute("lateralSpeed", &linearVelocity.y())!=TIXML_SUCCESS) {
        printf("Could not find lateralSpeed of knot!\n");
        return false;
      }
      if (child->QueryDoubleAttribute("turningSpeed", &localAngularVelocity.z())!=TIXML_SUCCESS) {
        printf("Could not find turningSpeed of knot!\n");
        return false;
      }
      if (!isInterpolatingTime_) {
         t *=  cycleDuration_;
      }
      linearVelocityTrajectory_.addKnot(t, linearVelocity);
      localAngularVelocityTrajectory_.addKnot(t, localAngularVelocity);


   }
  return true;
}

} /* namespace loco */
