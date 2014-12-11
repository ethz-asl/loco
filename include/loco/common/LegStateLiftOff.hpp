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
 * LegStateLiftOff.hpp
 *
 *  Created on: Feb 26, 2014
 *      Author: gech
 */

#ifndef LOCO_LEGSTATELIFTOFF_HPP_
#define LOCO_LEGSTATELIFTOFF_HPP_

#include "loco/common/TypeDefs.hpp"
#include "loco/common/LegStateBase.hpp"

#include <Eigen/Core>

namespace loco {

//!  State of the leg at the event of lift-off
/*!
 */
class LegStateLiftOff : public loco::LegStateBase {
 public:
  LegStateLiftOff();
  virtual ~LegStateLiftOff();

  const Position& getPositionWorldToHipInWorldFrame() const;
  const Position& getPositionWorldToFootInWorldFrame() const;
  const Position& getPositionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame() const;


  void setPositionWorldToHipInWorldFrame(const Position& hipPositionInWorldFrame);
  void setPositionWorldToFootInWorldFrame(const Position& footPositionInWorldFrame);
  void setPositionWorldToHipOnTerrainAlongWorldZInWorldFrame(const Position& positionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame);


 protected:
  Position footPositionInWorldFrame_;
  Position hipPositionInWorldFrame_;
  Position positionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame_;

};

} /* namespace loco */

#endif /* LOCO_LEGSTATELIFTOFF_HPP_ */
