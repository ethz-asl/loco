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
 * MissionControlSpeedFilter.hpp
 *
 *  Created on: Mar 7, 2014
 *      Author: gech
 */

#ifndef LOCO_MissionControlSpeedFilter_HPP_
#define LOCO_MissionControlSpeedFilter_HPP_

#include "loco/mission_control/MissionControlBase.hpp"
#include "loco/temp_helpers/FilteredVariable.hpp"

namespace loco {

class MissionControlSpeedFilter: public MissionControlBase {
 public:
  MissionControlSpeedFilter();
  virtual ~MissionControlSpeedFilter();

  virtual bool initialize(double dt);
  virtual bool advance(double dt);
  const Twist& getDesiredBaseTwistInHeadingFrame() const;
  const Twist& getMaximumBaseTwistInHeadingFrame() const;
  void setUnfilteredDesiredBaseTwistInHeadingFrame(const Twist& twist);
  void setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(const Position& position);

  virtual bool loadParameters(const TiXmlHandle& handle);

  bool setToInterpolated(const MissionControlBase& missionController1, const MissionControlBase& missionController2, double t);

  const Position& getDesiredPositionOffsetInWorldFrame() const;
  const Position& getMinimalPositionOffsetInWorldFrame() const;
  const Position& getMaximalPositionOffsetInWorldFrame() const;



  const RotationQuaternion& getDesiredOrientationOffset() const;
  void setUnfilteredDesiredOrientationOffset(const RotationQuaternion& desiredOrientationHeadingToBase);

  const EulerAnglesZyx& getMinimalDesiredOrientationOffset() const;
  const EulerAnglesZyx& getMaximalDesiredOrientationOffset() const;

  friend std::ostream& operator << (std::ostream& out, const MissionControlSpeedFilter& speedFilter);

 protected:
  Twist baseTwistInHeadingFrame_;
  Twist unfilteredBaseTwistInHeadingFrame_;
  Twist maximumBaseTwistInHeadingFrame_;
  Position desiredPositionOffsetInWorldFrame_;
  Position minimalDesiredPositionOffsetInWorldFrame_;
  Position maximalDesiredPositionOffsetInWorldFrame_;
  RotationQuaternion desiredOrientationHeadingToBase_;
  EulerAnglesZyx minimalOrientationHeadingToBase_;
  EulerAnglesZyx maximalOrientationHeadingToBase_;


  //! filtered speeds [sagittal; coronal; turning]
  FilteredDouble filteredVelocities_[3];



};

} /* namespace loco */

#endif /* LOCO_MissionControlSpeedFilter_HPP_ */
