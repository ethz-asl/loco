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
 * TuningWithSliderboardLocoDynamicGait.cpp
 *
 *  Created on: Apr 1, 2014
 *      Author: gech
 */

#include "loco/tools/TuningWithSliderboardLocoDynamicGait.hpp"

namespace loco {

TuningWithSliderboardLocoDynamicGait::TuningWithSliderboardLocoDynamicGait(robotUtils::Sliderboard* sliderboard) :
    TuningWithSliderboardBase(sliderboard),
    locomotionController_(nullptr)
{

}
TuningWithSliderboardLocoDynamicGait::TuningWithSliderboardLocoDynamicGait(robotUtils::Sliderboard* sliderboard, LocomotionControllerDynamicGait* locomotionController) :
    TuningWithSliderboardBase(sliderboard),
    locomotionController_(locomotionController)
{


}

void TuningWithSliderboardLocoDynamicGait::setLocomotionController(LocomotionControllerDynamicGait* locomotionController) {
  locomotionController_ = locomotionController;
}

TuningWithSliderboardLocoDynamicGait::~TuningWithSliderboardLocoDynamicGait() {

}

bool TuningWithSliderboardLocoDynamicGait::initialize(double dt)
{
 if (locomotionController_ == nullptr) {
   return false;
 }

 if(!initializeVirtualModelController(dt)) {
   return false;
 }

 return true;
}

bool TuningWithSliderboardLocoDynamicGait::advance(double dt) {
  if(!advanceVirtualModelController(dt)) {
    return false;
  }
  return true;
}

bool TuningWithSliderboardLocoDynamicGait::initializeVirtualModelController(double dt)
{
  printf("init sliderboard\n");


  VirtualModelController* vmController = locomotionController_->getVirtualModelController();

  Eigen::Vector3d pGainsTranslation = vmController->getProportionalGainTranslation();
  Eigen::Vector3d dGainsTranslation = vmController->getDerivativeGainTranslation();
  Eigen::Vector3d fGainsTranslation = vmController->getFeedforwardGainTranslation();

  Eigen::Vector3d pGainsRotation = vmController->getProportionalGainRotation();
  Eigen::Vector3d dGainsRotation = vmController->getDerivativeGainRotation();
  Eigen::Vector3d fGainsRotation = vmController->getFeedforwardGainRotation();

 // VForce heading
 sliderboard_->setSliderName(26, "VF heading kp");
 sliderboard_->setMapLinearMinMidMax(26, 0.0, pGainsTranslation.x(), 1000.0);
 sliderboard_->setSliderName(18, "VF heading kd");
 sliderboard_->setMapLinearMinMidMax(18, 0.0, dGainsTranslation.x(), 120.0);
 sliderboard_->setSliderName(3, "VF heading kff");
 sliderboard_->setMapLinearMinMidMax(3, 0.0, fGainsTranslation.x(), 100.0);

 // VForce lateral
 sliderboard_->setSliderName(27, "VF lateral kp");
 sliderboard_->setMapLinearMinMidMax(27, 0.0, pGainsTranslation.y(), 1000.0);
 sliderboard_->setSliderName(19, "VF lateral kd");
 sliderboard_->setMapLinearMinMidMax(19, 0.0, dGainsTranslation.y(), 120.0);

 // VForce vertical
 sliderboard_->setSliderName(28, "VF vertical kp");
 sliderboard_->setMapLinearMinMidMax(28, 0.0, pGainsTranslation.z(), 1000.0);
 sliderboard_->setSliderName(20, "VF vertical kd");
 sliderboard_->setMapLinearMinMidMax(20, 0.0, dGainsTranslation.z(), 200.0);
// sliderboard_->setSliderName(12, "VF vertical kff");
// sliderboard_->setMapLinearMinMidMax(12, 0.0, fGainsTranslation.z(), 350.0);


 // roll gains
 sliderboard_->setSliderName(29, "VF roll kp");
 sliderboard_->setMapLinearMinMidMax(29, 0.0, pGainsRotation.x(), 400.0);
 sliderboard_->setSliderName(21, "VF roll kd");
 sliderboard_->setMapLinearMinMidMax(21, 0.0, dGainsRotation.x(), 50.0);

 // pitch gain
 sliderboard_->setSliderName(30, "VF pitch kp");
 sliderboard_->setMapLinearMinMidMax(30, 0.0, pGainsRotation.y(), 400.0);
 sliderboard_->setSliderName(22, "VF pitch kd");
 sliderboard_->setMapLinearMinMidMax(22, 0.0, dGainsRotation.y(), 50.0);

 // yaw gain
 sliderboard_->setSliderName(30, "VF yaw kp");
 sliderboard_->setMapLinearMinMidMax(31, 0.0, pGainsRotation.z(), 200.0);
 sliderboard_->setSliderName(22, "VF yaw kd");
 sliderboard_->setMapLinearMinMidMax(23, 0.0, dGainsRotation.z(), 300.0);


 sliderboard_->setSliderName(1, "gravity comp. percentage");
 sliderboard_->setMapLinearMinMidMax(1, 0.5, 1.0, 1.5);

 return true;
}



bool TuningWithSliderboardLocoDynamicGait::advanceVirtualModelController(double dt) {
  VirtualModelController* vmController = locomotionController_->getVirtualModelController();
  double kp, kd, kff;

  // sagittal
  if (sliderboard_->getValue(26, kp) || sliderboard_->getValue(18, kd)
      || sliderboard_->getValue(3, kff)) {

    vmController->getGainsHeading(kp, kd, kff);
    sliderboard_->getValue(26, kp);
    sliderboard_->getValue(18, kd);
    sliderboard_->getValue(3, kff);
    printf("VMC sagittal: kp=%f\tkd=%f\tkff=%f\n", kp, kd, kff);
    vmController->setGainsHeading(kp, kd, kff);
  }
  // lateral
  if (sliderboard_->getValue(27, kp) || sliderboard_->getValue(19, kd)) {
    vmController->getGainsLateral(kp, kd, kff);
    sliderboard_->getValue(27, kp);
    sliderboard_->getValue(19, kd);
    printf("VMC lateral: kp=%f\tkd=%f\n", kp, kd);
    vmController->setGainsLateral(kp, kd, kff);
  }
  // vertical
  if (sliderboard_->getValue(28, kp) || sliderboard_->getValue(20, kd) || sliderboard_->getValue(12, kff)) {
    vmController->getGainsVertical(kp, kd, kff);
    sliderboard_->getValue(28, kp);
    sliderboard_->getValue(20, kd);
    sliderboard_->getValue(12, kff);
    printf("VMC vertical: kp=%f\tkd=%f\tff=%f\n", kp, kd, kff);
    vmController->setGainsVertical(kp, kd, kff);
  }

  // roll
  if (sliderboard_->getValue(29, kp) || sliderboard_->getValue(21, kd) ) {
    vmController->getGainsRoll(kp, kd, kff);
    sliderboard_->getValue(29, kp);
    sliderboard_->getValue(21, kd);
    printf("VMC roll: kp=%f\tkd=%f\n", kp, kd);
    vmController->setGainsRoll(kp, kd, kff);
  }

  // pitch
  if (sliderboard_->getValue(30, kp) || sliderboard_->getValue(22, kd) ) {
    vmController->getGainsPitch(kp, kd, kff);
    sliderboard_->getValue(30, kp);
    sliderboard_->getValue(22, kd);
    printf("VMC pitch: kp=%f\tkd=%f\n", kp, kd);
    vmController->setGainsPitch(kp, kd, kff);
  }

  // yaw
  if (sliderboard_->getValue(31, kp) || sliderboard_->getValue(23, kd) ) {
    vmController->getGainsYaw(kp, kd, kff);
    sliderboard_->getValue(31, kp);
    sliderboard_->getValue(23, kd);
    printf("VMC yaw: kp=%f\tkd=%f\n", kp, kd);
    vmController->setGainsYaw(kp, kd, kff);
  }


  if (sliderboard_->getValue(1, kp)  ) {
     printf("gravity comp. percentage: v=%f\n", kp);
     vmController->setGravityCompensationForcePercentage(kp);
   }
  return true;
}

} /* namespace loco */
