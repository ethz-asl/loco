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
 * GeneralizedStateStarlETH.hpp
 *
 *  Created on: Apr 4, 2014
 *      Author: gech
 */

#ifndef LOCO_GENERALIZEDSTATESTARLETH_HPP_
#define LOCO_GENERALIZEDSTATESTARLETH_HPP_

#include "loco/common/GeneralizedStateBase.hpp"
#include "loco/common/TypeDefsStarlETH.hpp"

#include <tinyxml.h>

namespace loco {

class GeneralizedStateStarlETH: public GeneralizedStateBase {
 public:
  GeneralizedStateStarlETH();
  virtual ~GeneralizedStateStarlETH();

  const GeneralizedCoordinates& getGeneralizedCoordinates() const;
  const GeneralizedVelocities& getGeneralizedVelocities() const;
  const GeneralizedAccelerations& getGeneralizedAccelerations() const;
  void setJointPositionsSymmetricToLeftForeLeg(double angleHAA, double angleHFE, double angleKFE);
  void setJointVelocitiesSymmetricToLeftForeLeg(double velocityHAA, double velocityHFE, double velocityKFE);
  void setJointPositionsForLeg(int iLeg, const Eigen::Vector3d& angles);
  void setPositionWorldToBaseInWorldFrame(const Position& worldToBaseInWorldFrame);
  void setOrientationWorldToBase(const RotationQuaternion& orientationWorldToBase);
  bool loadParameters(const TiXmlHandle& handle);
 protected:
  GeneralizedCoordinates generalizedCoordinates_;
  GeneralizedVelocities generalizedVelocities_;
  GeneralizedAccelerations generalizedAccelerations_;


};

} /* namespace loco */

#endif /* LOCO_GENERALIZEDSTATESTARLETH_HPP_ */
