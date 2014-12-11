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
 * TorsoStarlETH.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef LOCO_STATEDYNAMICGAIT_HPP_
#define LOCO_STATEDYNAMICGAIT_HPP_


#include "loco/common/TorsoBase.hpp"
#include "loco/common/TorsoPropertiesStarlETH.hpp"

#include "kindr/poses/PoseDiffEigen.hpp"
#include "kindr/poses/PoseEigen.hpp"
#include <Eigen/Core>

#include "starlethModel/RobotModel.hpp"

namespace loco {

//! Torso of StarlETH
/*! This should be used only as a data container
 *
 */
class TorsoStarlETH: public TorsoBase {
 public:
  TorsoStarlETH(robotModel::RobotModel* robotModel);
  virtual ~TorsoStarlETH();

  virtual double getStridePhase();
  virtual void setStridePhase(double stridePhase);

  virtual bool initialize(double dt);
  virtual bool advance(double dt);


  virtual TorsoStateMeasured& getMeasuredState();
  virtual TorsoStateDesired& getDesiredState();
  virtual const TorsoStateDesired& getDesiredState() const;

  virtual TorsoPropertiesBase& getProperties();

  friend std::ostream& operator << (std::ostream& out, const TorsoStarlETH& torso);
protected:
  robotModel::RobotModel* robotModel_;

  TorsoStateMeasured stateMeasured_;
  TorsoStateDesired stateDesired_;
  TorsoPropertiesStarlETH properties_;
  double stridePhase_;



};

} /* namespace loco */

#endif /* LOCO_STATEDYNAMICGAIT_HPP_ */
