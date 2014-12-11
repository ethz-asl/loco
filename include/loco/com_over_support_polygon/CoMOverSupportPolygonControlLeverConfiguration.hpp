/********************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014,  C. Dario Bellicoso, Christian Gehring, Péter Fankhauser, Stelian Coros
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
* @file     CoMOverSupportPolygonControlLeverConfiguration.hpp
* @author   C. Dario Bellicoso
* @date     Oct 7, 2014
* @brief
*/

#ifndef LOCO_COMOVERSUPPORTPOLYGONCONTROLLEVERCONFIGURATION_HPP_
#define LOCO_COMOVERSUPPORTPOLYGONCONTROLLEVERCONFIGURATION_HPP_

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlBase.hpp"

namespace loco {

class CoMOverSupportPolygonControlLeverConfiguration: public CoMOverSupportPolygonControlBase {
 public:
  CoMOverSupportPolygonControlLeverConfiguration(LegGroup* legs, TorsoBase* torso, TerrainModelBase* terrainModel);
  virtual ~CoMOverSupportPolygonControlLeverConfiguration();

  virtual bool setToInterpolated(const CoMOverSupportPolygonControlBase& supportPolygon1, const CoMOverSupportPolygonControlBase& supportPolygon2, double t);

  virtual void advance(double dt);

 protected:
  TorsoBase* torso_;
  TerrainModelBase* terrainModel_;

  Position positionCenterToForeHindSupportFeetInControlFrame_[2];
  Position positionWorldToCenterInWorldFrame_;

};

} /* namespace loco */


#endif /* LOCO_COMOVERSUPPORTPOLYGONCONTROLLEVERCONFIGURATION_HPP_ */
