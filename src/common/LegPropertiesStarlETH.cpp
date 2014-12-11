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
 * LegPropertiesStarlETH.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/common/LegPropertiesStarlETH.hpp"

namespace loco {

LegPropertiesStarlETH::LegPropertiesStarlETH(int iLeg, robotModel::RobotModel* robotModel)
    : LegPropertiesBase(),
      iLeg_(iLeg),
      robotModel_(robotModel)
{

}

LegPropertiesStarlETH::~LegPropertiesStarlETH()
{

}

bool LegPropertiesStarlETH::initialize(double dt)
{
    return true;
}

bool LegPropertiesStarlETH::advance(double dt)
{
  double mass = robotModel_->params().hip_.m + robotModel_->params().thigh_.m + robotModel_->params().shank_.m;
  setMass(mass);

  Position positionBaseToCenterOfMassInBaseFrame = Position(
     (robotModel_->kin().getJacobianTByLeg_Base2HipCoG_CSmb(iLeg_)->getPos() * robotModel_->params().hip_.m
    + robotModel_->kin().getJacobianTByLeg_Base2ThighCoG_CSmb(iLeg_)->getPos() * robotModel_->params().thigh_.m
    + robotModel_->kin().getJacobianTByLeg_Base2ShankCoG_CSmb(iLeg_)->getPos() * robotModel_->params().shank_.m) / mass);
  setBaseToCenterOfMassPositionInBaseFrame(positionBaseToCenterOfMassInBaseFrame);

  return true;
}

double LegPropertiesStarlETH::getLegLength() {
  return std::abs(robotModel_->params().hip_.l) + std::abs(robotModel_->params().thigh_.l) + std::abs(robotModel_->params().shank_.l);
}

} /* namespace loco */
