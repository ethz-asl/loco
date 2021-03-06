/*******************************************************************************************
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
 * TorsoControlStaticGait.hpp
 *
 *  Created on: Oct 9, 2014
 *      Author: dario
 */

#ifndef LOCO_TORSOCONTROLSTATICGAIT_HPP_
#define LOCO_TORSOCONTROLSTATICGAIT_HPP_


#include "loco/torso_control/TorsoControlGaitContainer.hpp"
#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlStaticGait.hpp"

namespace loco {

class TorsoControlStaticGait: public TorsoControlGaitContainer {
 public:
  TorsoControlStaticGait(LegGroup* legs, TorsoBase* torso, TerrainModelBase* terrain);
  virtual ~TorsoControlStaticGait();

  virtual bool initialize(double dt);
  virtual bool advance(double dt);

  virtual bool loadParameters(const TiXmlHandle& handle);

  virtual void setIsInStandConfiguration(bool isInStandConfiguration);
  virtual bool getIsInStandConfiguration() const;

  virtual bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t);

 protected:
  bool isInStandConfiguration_;

};

} /* namespace loco */


#endif /* LOCO_TORSOCONTROLSTATICGAIT_HPP_ */
