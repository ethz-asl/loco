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
 * TerrainModelSimulation.cpp
 *
 *  Created on: May 5, 2014
 *      Author: Peter Fankhauser
 */

#include "loco/common/TerrainModelSimulation.hpp"
#include "starlethModel/RobotModel.hpp"

namespace loco {

TerrainModelSimulation::TerrainModelSimulation(robotTerrain::TerrainBase* terrain)
    : TerrainModelBase(),
      terrain_(terrain)
{

}

TerrainModelSimulation::~TerrainModelSimulation()
{

}

bool TerrainModelSimulation::initialize(double dt) {
  return true;
}

bool TerrainModelSimulation::getNormal(const loco::Position& position, loco::Vector& normal) const
{
  robotModel::VectorP tempPosition = position.vector();
  terrain_->getNormal(tempPosition, normal.vector());
  return true;
}

bool TerrainModelSimulation::getHeight(loco::Position& positionWorldToLocationInWorldFrame) const
{
  robotModel::VectorP tempPosition = positionWorldToLocationInWorldFrame.vector();
  if(!terrain_->getHeight(tempPosition)) {
    positionWorldToLocationInWorldFrame.z() = 0.0;
    return false;
  }
  positionWorldToLocationInWorldFrame.z() = tempPosition.z();
  return true;
}

bool TerrainModelSimulation::getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const {
  robotModel::VectorP tempPosition = positionWorldToLocationInWorldFrame.vector();
  if(!terrain_->getHeight(tempPosition)) {
    heightInWorldFrame = 0.0;
    return false;
  }
  heightInWorldFrame = tempPosition.z();
  return true;
}

bool TerrainModelSimulation::getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const {
  assert(false); // not yet implemented!
  return false;
}




} /* namespace loco */
