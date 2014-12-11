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
 * TorsoControlDynamicGaitFreePlane.hpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Dario Bellicoso
 */

#ifndef LOCO_TORSOCONTROLDYNAMICGAITFREEPLANE_HPP_
#define LOCO_TORSOCONTROLDYNAMICGAITFREEPLANE_HPP_

#include "loco/torso_control/TorsoControlGaitContainer.hpp"
#include "robotUtils/filters/FirstOrderFilter.hpp"

namespace loco {

class TorsoControlDynamicGaitFreePlane: public TorsoControlGaitContainer {

  public:
    enum AdaptToTerrain {CompleteAdaption, SaturatedLinearAdaption};
    TorsoControlDynamicGaitFreePlane(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain);
    virtual ~TorsoControlDynamicGaitFreePlane();


    virtual bool initialize(double dt);
    virtual bool advance(double dt);

    virtual bool loadParameters(const TiXmlHandle& handle);

    virtual bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t);


  protected:
    //--- Terrain adaptation parameters
    double maxDesiredPitchRadians_;
    double desiredPitchSlope_;
    double maxDesiredRollRadians_;
    double desiredRollSlope_;
    AdaptToTerrain adaptToTerrain_;
    //---

    //--- Height offset parameters
    //double maxHeightOffset_;
    robotUtils:: FirstOrderFilter* firstOrderFilter_;
    //---

    template <typename T> int sgn(T val);

    void getDesiredBasePitchFromTerrainPitch(const double terrainPitch, double& desiredBasePitch);
    void getDesiredBaseRollFromTerrainRoll(const double terrainRoll, double& desiredBaseRoll);

};

} /* namespace loco */


#endif /* LOCO_TORSOCONTROLDYNAMICGAITFREEPLANE_HPP_ */
