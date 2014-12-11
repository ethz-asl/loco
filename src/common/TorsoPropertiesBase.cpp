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
 * TorsoPropertiesBase.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/common/TorsoPropertiesBase.hpp"

namespace loco {

TorsoPropertiesBase::TorsoPropertiesBase()
    : mass_(0.0),
      positionBaseToCenterOfMassInBaseFrame_()
{

}

TorsoPropertiesBase::~TorsoPropertiesBase()
{

}

double TorsoPropertiesBase::getMass() const
{
  return mass_;
}

void TorsoPropertiesBase::setMass(double mass)
{
  mass_ = mass;
}

const Position& TorsoPropertiesBase::getBaseToCenterOfMassPositionInBaseFrame() const
{
  return positionBaseToCenterOfMassInBaseFrame_;
}

void TorsoPropertiesBase::setBaseToCenterOfMassPositionInBaseFrame(const Position& centerOfMassInBaseFrame)
{
  positionBaseToCenterOfMassInBaseFrame_ = centerOfMassInBaseFrame;
}

const LinearAcceleration& TorsoPropertiesBase::getGravity() const
{
  return gravity_;
}

void TorsoPropertiesBase::setGravity(const LinearAcceleration& gravity)
{
  gravity_ = gravity;
  gravityAxisInWorldFrame_ = Vector(gravity).normalized();
}

const Vector& TorsoPropertiesBase::getHeadingAxisInBaseFrame()
{
  return headingAxisInBaseFrame_;
}

const Vector& TorsoPropertiesBase::getLateralAxisInBaseFrame()
{
  return lateralAxis_;
}

const Vector& TorsoPropertiesBase::getVerticalAxisInBaseFrame()
{
  return verticalAxisInBaseFrame_;
}

void TorsoPropertiesBase::setHeadingAxisInBaseFrame(const Vector& axis)
{
  headingAxisInBaseFrame_ = axis;
}

void TorsoPropertiesBase::setLateralAxisInBaseFrame(const Vector& axis)
{
  lateralAxis_ = axis;
}

void TorsoPropertiesBase::setVerticalAxisInBaseFrame(const Vector& axis)
{
  verticalAxisInBaseFrame_ = axis;
}

const Vector& TorsoPropertiesBase::getGravityAxisInWorldFrame() {
  return gravityAxisInWorldFrame_;
}




} /* namespace loco */
