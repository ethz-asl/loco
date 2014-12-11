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
* @file     CoMOverSupportPolygonControlBase.hpp
* @author   Christian Gehring, C. Dario Bellicoso
* @date     Oct 7, 2014
* @brief
*/
#ifndef LOCO_COMOVERSUPPORTPOLYGONCONTROLBASE_HPP_
#define LOCO_COMOVERSUPPORTPOLYGONCONTROLBASE_HPP_

#include "tinyxml.h"

#include <Eigen/Core>
#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/TypeDefs.hpp"

namespace loco {

class CoMOverSupportPolygonControlBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CoMOverSupportPolygonControlBase(LegGroup* legs);
  virtual ~CoMOverSupportPolygonControlBase();

  virtual void advance(double dt) = 0;

  /*! Gets the error vector from the center of all feet to the desired weighted location of the center of all feet in world coordinates
   * @param legs  references to the legs
   * @return error vector expressed in world frame
   */
  virtual const loco::Position& getPositionWorldToDesiredCoMInWorldFrame() const = 0;

  /*! Loads the parameters from the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  virtual bool loadParameters(TiXmlHandle &hParameterSet);

  /*! Stores the current paramters in the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  virtual bool saveParameters(TiXmlHandle &hParameterSet);

  /*! Computes an interpolated version of the two support polygon settings passed in as parameters.
   *  if t is 0, the current setting is set to supportPolygon1, 1 -> supportPolygon2, and values in between
   *  correspond to interpolated parameter set.
   * @param supportPolygon1
   * @param supportPolygon2
   * @param t   interpolation parameter
   * @return  true if successful
   */
  virtual bool setToInterpolated(const CoMOverSupportPolygonControlBase& supportPolygon1, const CoMOverSupportPolygonControlBase& supportPolygon2, double t) = 0;


  double getMinSwingLegWeight() const;
  double getStartShiftAwayFromLegAtStancePhase() const;
  double getStartShiftTowardsLegAtSwingPhase() const;
  double getLateralOffset() const;
  double getHeadingOffset() const;


 protected:
  Position positionWorldToDesiredCoMInWorldFrame_;

  LegGroup* legs_;
  //! this is the minimum weight any leg can have... if this is zero,then the COM will try to be right at center of the support polygon [0,1]
  double minSwingLegWeight_;

  //! this is the point in the stance phase when the body should start shifting away from the leg... when the stance phase is 1, the leg weight will be minimum
  double startShiftAwayFromLegAtStancePhase_;

  //! this is the point in the swing phase when the body should start shifting back towards the leg... when the swing phase is 1, the weight is back full on the leg
  double startShiftTowardsLegAtSwingPhase_;

  double lateralOffset_;
  double headingOffset_;

};

} /* namespace loco */


#endif /* LOCO_COMOVERSUPPORTPOLYGONCONTROLBASE_HPP_ */
