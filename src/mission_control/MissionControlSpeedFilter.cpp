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
 * MissionControlSpeedFilter.cpp
 *
 *  Created on: Mar 7, 2014
 *      Author: gech
 */

#include "loco/mission_control/MissionControlSpeedFilter.hpp"
#include <limits>
#include "loco/temp_helpers/math.hpp"

namespace loco {

MissionControlSpeedFilter::MissionControlSpeedFilter()
:
  baseTwistInHeadingFrame_(),
  maximumBaseTwistInHeadingFrame_(LinearVelocity(std::numeric_limits<LinearVelocity::Scalar>::max(),
                                                 std::numeric_limits<LinearVelocity::Scalar>::max(),
                                                 std::numeric_limits<LinearVelocity::Scalar>::max()),
                                  LocalAngularVelocity(std::numeric_limits<LinearVelocity::Scalar>::max(),
                                                       std::numeric_limits<LinearVelocity::Scalar>::max(),
                                                       std::numeric_limits<LinearVelocity::Scalar>::max()) )
{
  filteredVelocities_[0].setAlpha(0.005);
  filteredVelocities_[1].setAlpha(0.05);
  filteredVelocities_[2].setAlpha(0.05);
}

MissionControlSpeedFilter::~MissionControlSpeedFilter() {

}

bool MissionControlSpeedFilter::initialize(double dt) {

  return true;
}

template<typename T>
T mapInRange (T x, T in_min, T in_max, T out_min, T out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


bool MissionControlSpeedFilter::advance(double dt) {
  const double maxHeadingVel = maximumBaseTwistInHeadingFrame_.getTranslationalVelocity().x();
  filteredVelocities_[0].update(unfilteredBaseTwistInHeadingFrame_.getTranslationalVelocity().x());
  double headingVel = filteredVelocities_[0].val();
//  boundToRange(&headingVel, -maxHeadingVel, maxHeadingVel);
  headingVel = mapInRange(headingVel, -1.0, 1.0, -maxHeadingVel, maxHeadingVel);

  const double maxLateralVel = maximumBaseTwistInHeadingFrame_.getTranslationalVelocity().y();
  filteredVelocities_[1].update(unfilteredBaseTwistInHeadingFrame_.getTranslationalVelocity().y());

  double lateralVel = filteredVelocities_[1].val();
//  boundToRange(&lateralVel, -maxLateralVel, maxLateralVel);
  lateralVel = mapInRange(lateralVel, -1.0, 1.0, -maxLateralVel, maxLateralVel);


  LinearVelocity linearVelocity(headingVel, lateralVel, 0.0);

  const double maxTurningVel = maximumBaseTwistInHeadingFrame_.getRotationalVelocity().z();
  filteredVelocities_[2].update(unfilteredBaseTwistInHeadingFrame_.getRotationalVelocity().z());
  double turningVel = filteredVelocities_[2].val();
  //boundToRange(&turningVel, -maxTurningVel, maxTurningVel);
  turningVel = mapInRange(turningVel, -1.0, 1.0, -maxTurningVel, maxTurningVel);

  LocalAngularVelocity angularVelocity(0.0, 0.0, turningVel);

  baseTwistInHeadingFrame_ = Twist(linearVelocity, angularVelocity);


  /* -------- Position -------- */


//  desiredPositionOffsetInWorldFrame_.z() = 0.5*(maxHeight-minHeight)*(joyStick->getVertical()-minHeight) + maxHeight;
  boundToRange(&desiredPositionOffsetInWorldFrame_.x(), minimalDesiredPositionOffsetInWorldFrame_.x(), maximalDesiredPositionOffsetInWorldFrame_.x());
  boundToRange(&desiredPositionOffsetInWorldFrame_.y(), minimalDesiredPositionOffsetInWorldFrame_.y(), maximalDesiredPositionOffsetInWorldFrame_.y());
  boundToRange(&desiredPositionOffsetInWorldFrame_.z(), minimalDesiredPositionOffsetInWorldFrame_.z(), maximalDesiredPositionOffsetInWorldFrame_.z());

  /* -------- Orientation -------- */

  EulerAnglesZyx desEulerAnglesZyx(desiredOrientationHeadingToBase_);
  desEulerAnglesZyx.setUnique();
  EulerAnglesZyx minEulerAnglesZyx(minimalOrientationHeadingToBase_);
  minEulerAnglesZyx.setUnique();
  EulerAnglesZyx maxEulerAnglesZyx(maximalOrientationHeadingToBase_);
  maxEulerAnglesZyx.setUnique();
  double value = desEulerAnglesZyx.roll();
  boundToRange(&value, minEulerAnglesZyx.roll(), maxEulerAnglesZyx.roll());
  desEulerAnglesZyx.setRoll(value);
  value = desEulerAnglesZyx.pitch();
  boundToRange(&value, minEulerAnglesZyx.pitch(), maxEulerAnglesZyx.pitch());
  desEulerAnglesZyx.setPitch(value);
  value = desEulerAnglesZyx.yaw();
  boundToRange(&value, minEulerAnglesZyx.yaw(), maxEulerAnglesZyx.yaw());
  desEulerAnglesZyx.setYaw(value);
  desiredOrientationHeadingToBase_(desEulerAnglesZyx.getUnique());

  return true;
}

void MissionControlSpeedFilter::setUnfilteredDesiredBaseTwistInHeadingFrame(const Twist& twist) {
  unfilteredBaseTwistInHeadingFrame_ = twist;
}

void MissionControlSpeedFilter::setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(const Position& position) {
  desiredPositionOffsetInWorldFrame_ = position;
}

const Twist& MissionControlSpeedFilter::getDesiredBaseTwistInHeadingFrame() const {
  return baseTwistInHeadingFrame_;
}

const RotationQuaternion& MissionControlSpeedFilter::getDesiredOrientationOffset() const {
  return desiredOrientationHeadingToBase_;
}
void MissionControlSpeedFilter::setUnfilteredDesiredOrientationOffset(const RotationQuaternion& desiredOrientationHeadingToBase) {
  desiredOrientationHeadingToBase_ = desiredOrientationHeadingToBase;
}


const Twist& MissionControlSpeedFilter::getMaximumBaseTwistInHeadingFrame() const {
  return maximumBaseTwistInHeadingFrame_;
}

bool MissionControlSpeedFilter::loadParameters(const TiXmlHandle& handle) {

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
    desiredPositionOffsetInWorldFrame_.x() = 0.0;
    desiredPositionOffsetInWorldFrame_.y() = 0.0;
    desiredPositionOffsetInWorldFrame_.z() = 0.42;
  } else {

  /* ---------------------------- Position ---------------------------- */

    TiXmlHandle hPosition(hConfiguration.FirstChild("Position"));
    pElem = hPosition.FirstChild("Initial").Element();
    if (!pElem) {
      printf("Could not find Configuration:Position:Initial\n");
      desiredPositionOffsetInWorldFrame_.x() = 0.0;
      desiredPositionOffsetInWorldFrame_.y() = 0.0;
      desiredPositionOffsetInWorldFrame_.z() = 0.42;
      return false;
    } else {
      if (pElem->QueryDoubleAttribute("x", &desiredPositionOffsetInWorldFrame_.x())!=TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Initial:x\n");
        desiredPositionOffsetInWorldFrame_.x() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("y", &desiredPositionOffsetInWorldFrame_.y())!=TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Initial:x\n");
        desiredPositionOffsetInWorldFrame_.y() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("z", &desiredPositionOffsetInWorldFrame_.z())!=TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Initial:z\n");
        desiredPositionOffsetInWorldFrame_.z() = 0.42;
      }
    }

    pElem = hPosition.FirstChild("Minimal").Element();
    if (!pElem) {
     printf("Could not find Configuration:Position:Minimal\n");
     minimalDesiredPositionOffsetInWorldFrame_.x() = 0.0;
     minimalDesiredPositionOffsetInWorldFrame_.y() = 0.0;
     minimalDesiredPositionOffsetInWorldFrame_.z() = 0.24;
     return false;
    } else {
     if (pElem->QueryDoubleAttribute("x", &minimalDesiredPositionOffsetInWorldFrame_.x())!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Position:Minimal:x\n");
       minimalDesiredPositionOffsetInWorldFrame_.x() = 0.0;
     }
     if (pElem->QueryDoubleAttribute("y", &minimalDesiredPositionOffsetInWorldFrame_.y())!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Position:Minimal:x\n");
       minimalDesiredPositionOffsetInWorldFrame_.y() = 0.0;
     }
     if (pElem->QueryDoubleAttribute("z", &minimalDesiredPositionOffsetInWorldFrame_.z())!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Position:Minimal:z\n");
       minimalDesiredPositionOffsetInWorldFrame_.z() = 0.24;
     }
    }

    pElem = hPosition.FirstChild("Maximal").Element();
    if (!pElem) {
     printf("Could not find Configuration:Position:Maximal\n");
     maximalDesiredPositionOffsetInWorldFrame_.x() = 0.0;
     maximalDesiredPositionOffsetInWorldFrame_.y() = 0.0;
     maximalDesiredPositionOffsetInWorldFrame_.z() = 0.44;
     return false;
    } else {
     if (pElem->QueryDoubleAttribute("x", &maximalDesiredPositionOffsetInWorldFrame_.x())!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Position:Maximal:x\n");
       maximalDesiredPositionOffsetInWorldFrame_.x() = 0.0;
     }
     if (pElem->QueryDoubleAttribute("y", &maximalDesiredPositionOffsetInWorldFrame_.y())!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Position:Maximal:x\n");
       maximalDesiredPositionOffsetInWorldFrame_.y() = 0.0;
     }
     if (pElem->QueryDoubleAttribute("z", &maximalDesiredPositionOffsetInWorldFrame_.z())!=TIXML_SUCCESS) {
       printf("Could not find Configuration:Position:Maximal:z\n");
       maximalDesiredPositionOffsetInWorldFrame_.z() = 0.44;
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

bool MissionControlSpeedFilter::setToInterpolated(const MissionControlBase& missionController1, const MissionControlBase& missionController2, double t) {
  const MissionControlSpeedFilter& controller1 = static_cast<const MissionControlSpeedFilter& >(missionController1);
  const MissionControlSpeedFilter& controller2 = static_cast<const MissionControlSpeedFilter& >(missionController2);
  maximumBaseTwistInHeadingFrame_.getTranslationalVelocity().x() = linearlyInterpolate(controller1.getMaximumBaseTwistInHeadingFrame().getTranslationalVelocity().x(),
                                                                                       controller2.getMaximumBaseTwistInHeadingFrame().getTranslationalVelocity().x(), 0.0, 1.0, t);
  maximumBaseTwistInHeadingFrame_.getTranslationalVelocity().y() = linearlyInterpolate(controller1.getMaximumBaseTwistInHeadingFrame().getTranslationalVelocity().y(),
                                                                                       controller2.getMaximumBaseTwistInHeadingFrame().getTranslationalVelocity().y(), 0.0, 1.0, t);
  maximumBaseTwistInHeadingFrame_.getTranslationalVelocity().z() = linearlyInterpolate(controller1.getMaximumBaseTwistInHeadingFrame().getTranslationalVelocity().z(),
                                                                                       controller2.getMaximumBaseTwistInHeadingFrame().getTranslationalVelocity().z(), 0.0, 1.0, t);
  maximumBaseTwistInHeadingFrame_.getRotationalVelocity().z() = linearlyInterpolate(controller1.getMaximumBaseTwistInHeadingFrame().getRotationalVelocity().z(),
                                                                                       controller2.getMaximumBaseTwistInHeadingFrame().getRotationalVelocity().z(), 0.0, 1.0, t);


  minimalDesiredPositionOffsetInWorldFrame_ = linearlyInterpolate(controller1.getMinimalPositionOffsetInWorldFrame(),
                                                                           controller2.getMinimalPositionOffsetInWorldFrame(), 0.0, 1.0, t);

  maximalDesiredPositionOffsetInWorldFrame_ = linearlyInterpolate(controller1.getMaximalPositionOffsetInWorldFrame(),
                                                                           controller2.getMaximalPositionOffsetInWorldFrame(), 0.0, 1.0, t);


  return true;
}


const Position& MissionControlSpeedFilter::getDesiredPositionOffsetInWorldFrame() const {
  return desiredPositionOffsetInWorldFrame_;
}
const Position& MissionControlSpeedFilter::getMinimalPositionOffsetInWorldFrame() const {
  return minimalDesiredPositionOffsetInWorldFrame_;
}

const Position& MissionControlSpeedFilter::getMaximalPositionOffsetInWorldFrame() const {
  return maximalDesiredPositionOffsetInWorldFrame_;
}

const EulerAnglesZyx& MissionControlSpeedFilter::getMinimalDesiredOrientationOffset() const {
  return minimalOrientationHeadingToBase_;
}
const EulerAnglesZyx& MissionControlSpeedFilter::getMaximalDesiredOrientationOffset() const {
  return maximalOrientationHeadingToBase_;
}


std::ostream& operator << (std::ostream& out, const MissionControlSpeedFilter& speedFilter) {
  return out;
}



} /* namespace loco */
