/*****************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Christian Gehring, Péter Fankhauser, C. Dario Bellicoso, Stelian Coros
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
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
  maximumBaseTwistInHeadingFrame_(LinearVelocity(std::numeric_limits<LinearVelocity::Scalar>::max(), std::numeric_limits<LinearVelocity::Scalar>::max(), std::numeric_limits<LinearVelocity::Scalar>::max()), LocalAngularVelocity(std::numeric_limits<LinearVelocity::Scalar>::max(), std::numeric_limits<LinearVelocity::Scalar>::max(), std::numeric_limits<LinearVelocity::Scalar>::max()) ),
  desiredPositionMiddleOfFeetToBaseInWorldFrame_(0.0, 0.0, 0.42),
  minimalPositionMiddleOfFeetToBaseInWorldFrame_(0.0, 0.0, 0.24),
  maximalPositionMiddleOfFeetToBaseInWorldFrame_(0.0, 0.0, 0.44),
  desiredOrientationHeadingToBase_(),
  minimalOrientationHeadingToBase_(),
  maximalOrientationHeadingToBase_()
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

bool MissionControlJoystick::advance(double dt) {
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


  /* -------- Position -------- */

//  desiredPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.5*(maxHeight-minHeight)*(joyStick->getVertical()-minHeight) + maxHeight;
  desiredPositionMiddleOfFeetToBaseInWorldFrame_.x() = interpolateJoystickAxis(joyStick->getSagittal(), minimalPositionMiddleOfFeetToBaseInWorldFrame_.x(), maximalPositionMiddleOfFeetToBaseInWorldFrame_.x());
  boundToRange(&desiredPositionMiddleOfFeetToBaseInWorldFrame_.x(), minimalPositionMiddleOfFeetToBaseInWorldFrame_.x(), maximalPositionMiddleOfFeetToBaseInWorldFrame_.x());
  desiredPositionMiddleOfFeetToBaseInWorldFrame_.y() = interpolateJoystickAxis(joyStick->getCoronal(), minimalPositionMiddleOfFeetToBaseInWorldFrame_.y(), maximalPositionMiddleOfFeetToBaseInWorldFrame_.y());
  boundToRange(&desiredPositionMiddleOfFeetToBaseInWorldFrame_.y(), minimalPositionMiddleOfFeetToBaseInWorldFrame_.y(), maximalPositionMiddleOfFeetToBaseInWorldFrame_.y());
  desiredPositionMiddleOfFeetToBaseInWorldFrame_.z() = interpolateJoystickAxis(joyStick->getVertical(), minimalPositionMiddleOfFeetToBaseInWorldFrame_.z(), maximalPositionMiddleOfFeetToBaseInWorldFrame_.z());
  boundToRange(&desiredPositionMiddleOfFeetToBaseInWorldFrame_.z(), minimalPositionMiddleOfFeetToBaseInWorldFrame_.z(), maximalPositionMiddleOfFeetToBaseInWorldFrame_.z());

  /* -------- Orientation -------- */
//  EulerAnglesZyx desEulerAnglesZyx;
//  desEulerAnglesZyx.setRoll(interpolateJoystickAxis(joyStick->getRoll(), minimalOrientationHeadingToBase_.roll(), maximalOrientationHeadingToBase_.roll()));
//  desEulerAnglesZyx.setRoll(boundToRange(desEulerAnglesZyx.roll(), minimalOrientationHeadingToBase_.roll(), maximalOrientationHeadingToBase_.roll()));
//  desEulerAnglesZyx.setPitch(interpolateJoystickAxis(joyStick->getPitch(), minimalOrientationHeadingToBase_.pitch(), maximalOrientationHeadingToBase_.pitch()));
//  desEulerAnglesZyx.setPitch(boundToRange(desEulerAnglesZyx.pitch(), minimalOrientationHeadingToBase_.pitch(), maximalOrientationHeadingToBase_.pitch()));
//  desEulerAnglesZyx.setYaw(interpolateJoystickAxis(joyStick->getYaw(), minimalOrientationHeadingToBase_.yaw(), maximalOrientationHeadingToBase_.yaw()));
//  desEulerAnglesZyx.setYaw(boundToRange(desEulerAnglesZyx.yaw(), minimalOrientationHeadingToBase_.yaw(), maximalOrientationHeadingToBase_.yaw()));
//  desiredOrientationHeadingToBase_(desEulerAnglesZyx);

  EulerAnglesZyx desEulerAnglesZyx;
  desEulerAnglesZyx.setRoll(interpolateJoystickAxis(joyStick->getCoronal(), minimalOrientationHeadingToBase_.getUnique().roll(), maximalOrientationHeadingToBase_.getUnique().roll()));
  desEulerAnglesZyx.setRoll(boundToRange(desEulerAnglesZyx.getUnique().roll(), minimalOrientationHeadingToBase_.roll(), maximalOrientationHeadingToBase_.getUnique().roll()));
  desEulerAnglesZyx.setPitch(interpolateJoystickAxis(joyStick->getSagittal(), minimalOrientationHeadingToBase_.getUnique().pitch(), maximalOrientationHeadingToBase_.getUnique().pitch()));
  desEulerAnglesZyx.setPitch(boundToRange(desEulerAnglesZyx.getUnique().pitch(), minimalOrientationHeadingToBase_.getUnique().pitch(), maximalOrientationHeadingToBase_.getUnique().pitch()));
  desEulerAnglesZyx.setYaw(interpolateJoystickAxis(joyStick->getYaw(), minimalOrientationHeadingToBase_.getUnique().yaw(), maximalOrientationHeadingToBase_.getUnique().yaw()));
  desEulerAnglesZyx.setYaw(boundToRange(desEulerAnglesZyx.getUnique().yaw(), minimalOrientationHeadingToBase_.getUnique().yaw(), maximalOrientationHeadingToBase_.getUnique().yaw()));
  desiredOrientationHeadingToBase_(desEulerAnglesZyx.getUnique());
  return true;
}

double MissionControlJoystick::interpolateJoystickAxis(double value, double minValue, double maxValue) {
  return 0.5*(maxValue-minValue)*(value+1.0) + minValue;
}

const Twist& MissionControlJoystick::getDesiredBaseTwistInHeadingFrame() const {
  return baseTwistInHeadingFrame_;
}
const Position& MissionControlJoystick::getDesiredPositionMiddleOfFeetToBaseInWorldFrame() const {
  return desiredPositionMiddleOfFeetToBaseInWorldFrame_;
}
const RotationQuaternion& MissionControlJoystick::getDesiredOrientationHeadingToBase() const {
  return desiredOrientationHeadingToBase_;
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

  /*  Configuration */
  TiXmlHandle hConfiguration(handle.FirstChild("LocomotionController").FirstChild("Mission").FirstChild("Configuration"));
  pElem = hConfiguration.Element();
  if (!pElem) {
    printf("Could not find Mission:Configuration\n");
    desiredPositionMiddleOfFeetToBaseInWorldFrame_.x() = 0.0;
    desiredPositionMiddleOfFeetToBaseInWorldFrame_.y() = 0.0;
    desiredPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.42;
  } else {

    /* ---------------------------- Position ---------------------------- */

    TiXmlHandle hPosition(hConfiguration.FirstChild("Position"));
    pElem = hPosition.FirstChild("Initial").Element();
    if (!pElem) {
      printf("Could not find Configuration:Position:Initial\n");
      desiredPositionMiddleOfFeetToBaseInWorldFrame_.x() = 0.0;
      desiredPositionMiddleOfFeetToBaseInWorldFrame_.y() = 0.0;
      desiredPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.42;
      return false;
    } else {
      if (pElem->QueryDoubleAttribute("x", &desiredPositionMiddleOfFeetToBaseInWorldFrame_.x())!=TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Initial:x\n");
        desiredPositionMiddleOfFeetToBaseInWorldFrame_.x() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("y", &desiredPositionMiddleOfFeetToBaseInWorldFrame_.y())!=TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Initial:x\n");
        desiredPositionMiddleOfFeetToBaseInWorldFrame_.y() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("z", &desiredPositionMiddleOfFeetToBaseInWorldFrame_.z())!=TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Initial:z\n");
        desiredPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.42;
      }
    }

    pElem = hPosition.FirstChild("Minimal").Element();
    if (!pElem) {
     printf("Could not find Configuration:Position:Minimal\n");
     minimalPositionMiddleOfFeetToBaseInWorldFrame_.x() = 0.0;
     minimalPositionMiddleOfFeetToBaseInWorldFrame_.y() = 0.0;
     minimalPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.24;
     return false;
    } else {
     if (pElem->QueryDoubleAttribute("x", &minimalPositionMiddleOfFeetToBaseInWorldFrame_.x())!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Position:Minimal:x\n");
       minimalPositionMiddleOfFeetToBaseInWorldFrame_.x() = 0.0;
     }
     if (pElem->QueryDoubleAttribute("y", &minimalPositionMiddleOfFeetToBaseInWorldFrame_.y())!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Position:Minimal:x\n");
       minimalPositionMiddleOfFeetToBaseInWorldFrame_.y() = 0.0;
     }
     if (pElem->QueryDoubleAttribute("z", &minimalPositionMiddleOfFeetToBaseInWorldFrame_.z())!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Position:Minimal:z\n");
       minimalPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.24;
     }
    }

    pElem = hPosition.FirstChild("Maximal").Element();
    if (!pElem) {
     printf("Could not find Configuration:Position:Maximal\n");
     maximalPositionMiddleOfFeetToBaseInWorldFrame_.x() = 0.0;
     maximalPositionMiddleOfFeetToBaseInWorldFrame_.y() = 0.0;
     maximalPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.44;
     return false;
    } else {
     if (pElem->QueryDoubleAttribute("x", &maximalPositionMiddleOfFeetToBaseInWorldFrame_.x())!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Position:Maximal:x\n");
       maximalPositionMiddleOfFeetToBaseInWorldFrame_.x() = 0.0;
     }
     if (pElem->QueryDoubleAttribute("y", &maximalPositionMiddleOfFeetToBaseInWorldFrame_.y())!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Position:Maximal:x\n");
       maximalPositionMiddleOfFeetToBaseInWorldFrame_.y() = 0.0;
     }
     if (pElem->QueryDoubleAttribute("z", &maximalPositionMiddleOfFeetToBaseInWorldFrame_.z())!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Position:Maximal:z\n");
       maximalPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.44;
     }
    }

    /* ---------------------------- Orientation ---------------------------- */
    TiXmlHandle hOrientation(hConfiguration.FirstChild("Orientation"));

    pElem = hOrientation.FirstChild("Initial").Element();
    EulerAnglesZyx desiredOrientationHeadingToBaseEulerAnglesZyx;
    if (!pElem) {
     printf("Could not find Configuration:Orientation:Initial\n");
     desiredOrientationHeadingToBaseEulerAnglesZyx.setX(0.0);
     desiredOrientationHeadingToBaseEulerAnglesZyx.setY(0.0);
     desiredOrientationHeadingToBaseEulerAnglesZyx.setZ(0.0);
     return false;
    } else {
     double value = 0.0;
     if (pElem->QueryDoubleAttribute("x", &value)!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Orientation:Initial:x\n");
       value = 0.0;
     }
     desiredOrientationHeadingToBaseEulerAnglesZyx.setX(value);

     if (pElem->QueryDoubleAttribute("y", &value)!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Orientation:Initial:x\n");
       value = 0.0;
     }
     desiredOrientationHeadingToBaseEulerAnglesZyx.setY(value);

     if (pElem->QueryDoubleAttribute("z", &value)!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Orientation:Initial:z\n");
       value = 0.0;
     }
     desiredOrientationHeadingToBaseEulerAnglesZyx.setZ(value);
    }
    desiredOrientationHeadingToBase_(desiredOrientationHeadingToBaseEulerAnglesZyx);


    /* Minimal */
    pElem = hOrientation.FirstChild("Minimal").Element();
    EulerAnglesZyx minimalOrientationHeadingToBaseEulerAnglesZyx;
    if (!pElem) {
     printf("Could not find Configuration:Orientation:Minimal\n");
     minimalOrientationHeadingToBaseEulerAnglesZyx.setX(0.0);
     minimalOrientationHeadingToBaseEulerAnglesZyx.setY(0.0);
     minimalOrientationHeadingToBaseEulerAnglesZyx.setZ(0.0);
     return false;
    } else {
     double value = 0.0;
     if (pElem->QueryDoubleAttribute("x", &value)!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Orientation:Minimal:x\n");
       value = 0.0;
     }
     minimalOrientationHeadingToBaseEulerAnglesZyx.setX(value);

     if (pElem->QueryDoubleAttribute("y", &value)!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Orientation:Minimal:x\n");
       value = 0.0;
     }
     minimalOrientationHeadingToBaseEulerAnglesZyx.setY(value);

     if (pElem->QueryDoubleAttribute("z", &value)!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Orientation:Minimal:z\n");
       value = 0.0;
     }
     minimalOrientationHeadingToBaseEulerAnglesZyx.setZ(value);
    }
    minimalOrientationHeadingToBase_(minimalOrientationHeadingToBaseEulerAnglesZyx);

    /* Maximal */
    pElem = hOrientation.FirstChild("Maximal").Element();
    EulerAnglesZyx maximalOrientationHeadingToBaseEulerAnglesZyx;
    if (!pElem) {
     printf("Could not find Configuration:Orientation:Maximal\n");
     maximalOrientationHeadingToBaseEulerAnglesZyx.setX(0.0);
     maximalOrientationHeadingToBaseEulerAnglesZyx.setY(0.0);
     maximalOrientationHeadingToBaseEulerAnglesZyx.setZ(0.0);
     return false;
    } else {
     double value = 0.0;
     if (pElem->QueryDoubleAttribute("x", &value)!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Orientation:Maximal:x\n");
       value = 0.0;
     }
     maximalOrientationHeadingToBaseEulerAnglesZyx.setX(value);

     if (pElem->QueryDoubleAttribute("y", &value)!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Orientation:Maximal:x\n");
       value = 0.0;
     }
     maximalOrientationHeadingToBaseEulerAnglesZyx.setY(value);

     if (pElem->QueryDoubleAttribute("z", &value)!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Orientation:Maximal:z\n");
       value = 0.0;
     }
     maximalOrientationHeadingToBaseEulerAnglesZyx.setZ(value);
    }
    maximalOrientationHeadingToBase_(maximalOrientationHeadingToBaseEulerAnglesZyx);

  }

  return true;
}

std::ostream& operator << (std::ostream& out, const MissionControlJoystick& joystick) {
  robotUtils::Joystick* joyStick = joystick.robotModel_->sensors().getJoystick();
  out << "joyStick position: " << joyStick->getSagittal() << " " << joyStick->getCoronal() << " " << joyStick->getVertical() << std::endl;
  out << "desiredPositionMiddleOfFeetToBaseInWorldFrame: " << joystick.desiredPositionMiddleOfFeetToBaseInWorldFrame_ << std::endl;
  out << "minimalPositionMiddleOfFeetToBaseInWorldFrame: " << joystick.minimalPositionMiddleOfFeetToBaseInWorldFrame_ << std::endl;
  out << "maximalPositionMiddleOfFeetToBaseInWorldFrame: " << joystick.maximalPositionMiddleOfFeetToBaseInWorldFrame_ << std::endl;
  out << "desiredOrientationHeadingToBase (EulerAnglesZyx): " << EulerAnglesZyx(joystick.desiredOrientationHeadingToBase_).getUnique() << std::endl;
  out << "minimalOrientationHeadingToBase (EulerAnglesZyx): " << EulerAnglesZyx(joystick.minimalOrientationHeadingToBase_).getUnique() << std::endl;
  out << "maximalOrientationHeadingToBase (EulerAnglesZyx): " << EulerAnglesZyx(joystick.maximalOrientationHeadingToBase_).getUnique() << std::endl;
  return out;
}

} /* namespace loco */
