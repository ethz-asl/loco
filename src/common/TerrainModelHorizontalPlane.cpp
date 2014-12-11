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
 * TerrainModelHorizontalPlane.cpp
 *
 *  Created on: Apr 4, 2014
 *      Author: gech
 */

#include "loco/common/TerrainModelHorizontalPlane.hpp"

namespace loco {

TerrainModelHorizontalPlane::TerrainModelHorizontalPlane() :
    TerrainModelBase(),
    heightInWorldFrame_(0.0),
    normalInWorldFrame_(loco::Vector::UnitZ()),
    frictionCoefficientBetweenTerrainAndFoot_(0.8)
{


}

TerrainModelHorizontalPlane::~TerrainModelHorizontalPlane() {

}

bool TerrainModelHorizontalPlane::initialize(double dt) {
  heightInWorldFrame_ = 0.0;
  normalInWorldFrame_ = loco::Vector::UnitZ();
  frictionCoefficientBetweenTerrainAndFoot_ = 0.6;
  return true;
}


bool TerrainModelHorizontalPlane::getNormal(const loco::Position& positionWorldToLocationInWorldFrame, loco::Vector& normalInWorldFrame) const
{
  normalInWorldFrame = normalInWorldFrame_;
  return true;
}

bool TerrainModelHorizontalPlane::getHeight(loco::Position& positionWorldToLocationInWorldFrame) const
{
  positionWorldToLocationInWorldFrame.z() = heightInWorldFrame_;
  return true;
}


bool TerrainModelHorizontalPlane::getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const {
  heightInWorldFrame = heightInWorldFrame_;
  return true;
}

void TerrainModelHorizontalPlane::setHeight(double heightInWorldFrame) {
  heightInWorldFrame_ = heightInWorldFrame;
}

bool TerrainModelHorizontalPlane::getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const {
  frictionCoefficient = frictionCoefficientBetweenTerrainAndFoot_;
  return true;
}

Position TerrainModelHorizontalPlane::getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(const Position& positionInWorldFrame)  const {
  Position position = positionInWorldFrame;
  getHeight(position);
  return position;
}


double TerrainModelHorizontalPlane::getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(const Position& positionInWorldFrame) const {
  double distance;
  getHeight(positionInWorldFrame, distance);
  return distance;
}


} /* namespace loco */
