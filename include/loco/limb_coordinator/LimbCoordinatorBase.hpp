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
/*!
* @file     LimbCoordinatorBase.hpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#ifndef LOCO_LIMBCOORDINATIONBASE_HPP_
#define LOCO_LIMBCOORDINATIONBASE_HPP_

#include "loco/gait_pattern/GaitPatternBase.hpp"
#include "tinyxml.h"


namespace loco {

class LimbCoordinatorBase {
 public:
  LimbCoordinatorBase();
  virtual ~LimbCoordinatorBase();

  virtual bool initialize(double dt) = 0;
  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual bool advance(double dt) = 0;

  virtual GaitPatternBase* getGaitPattern() = 0;


  virtual bool loadParameters(const TiXmlHandle& handle) = 0;

  /*! Computes an interpolated version of the two controllers passed in as parameters.
   *  If t is 0, the current setting is set to limbCoordinator1, 1 -> limbCoordinator2, and values in between
   *  correspond to interpolated parameter set.
   * @param limbCoordinator1
   * @param limbCoordinator2
   * @param t interpolation parameter
   * @returns true if successful
   */
  virtual bool setToInterpolated(const LimbCoordinatorBase& limbCoordinator1, const LimbCoordinatorBase& limbCoordinator2, double t);
};

} /* namespace loco */

#endif /* LOCO_LIMBCOORDINATIONBASE_HPP_ */
