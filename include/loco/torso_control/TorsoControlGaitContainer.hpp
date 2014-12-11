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
 * TorsoControlGaitContainer.hpp
 *
 *  Created on: Oct 29, 2014
 *      Author: Dario Bellicoso
 */

#ifndef LOCO_TORSOCONTROLGAITCONTAINER_HPP_
#define LOCO_TORSOCONTROLGAITCONTAINER_HPP_

#include "loco/torso_control/TorsoControlBase.hpp"

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlBase.hpp"
#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"
#include "loco/common/TerrainModelBase.hpp"

namespace loco {

class TorsoControlGaitContainer : public TorsoControlBase {

 public:
  TorsoControlGaitContainer(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain);
  virtual ~TorsoControlGaitContainer();

  virtual bool initialize(double dt);
  virtual bool advance(double dt);
  virtual bool loadParameters(const TiXmlHandle& handle);

  /*! Return the default fore torso height as defined in parameter file.
   * @returns the default height
   */
  virtual double getDesiredTorsoForeHeightAboveGroundInWorldFrameOffset() const;

  /*! Return the default hind torso height as defined in parameter file.
   * @returns the default height
   */
  virtual double getDesiredTorsoHindHeightAboveGroundInWorldFrameOffset() const;

  /*! Return default Center of Mass height as defined in parameter file.
   * @returns the default height
   */
  virtual double getDesiredTorsoCoMHeightAboveGroundInControlFrameOffset() const;

  /*! Set a position offset to the CoM in world frame (used by the mission controller to move the robot while standing)
   * @param[in] positionOffsetInWorldFrame The position offset
   */
  virtual void setDesiredPositionOffsetInWorldFrame(const Position& positionOffsetInWorldFrame);

  /*! Set an attitude offset to the CoM w.r.t. world frame (used by the mission controller to change the robot attitude while standing)
   * @param[in] orientationOffset The orientation offset
   */
  virtual void setDesiredOrientationOffset(const RotationQuaternion& orientationOffset);

  const CoMOverSupportPolygonControlBase& getCoMOverSupportPolygonControl() const;
  CoMOverSupportPolygonControlBase* getCoMControl();

 protected:
  LegGroup* legs_;
  TorsoBase* torso_;
  TerrainModelBase* terrain_;
  CoMOverSupportPolygonControlBase* comControl_;

  double headingDistanceFromForeToHindInBaseFrame_;
  double desiredTorsoForeHeightAboveGroundInWorldFrameOffset_;
  double desiredTorsoHindHeightAboveGroundInWorldFrameOffset_;
  double desiredTorsoCoMHeightAboveGroundInControlFrameOffset_;
  rbf::PeriodicRBF1DC1 desiredTorsoForeHeightAboveGroundInWorldFrame_;
  rbf::PeriodicRBF1DC1 desiredTorsoHindHeightAboveGroundInWorldFrame_;
  Position desiredPositionOffsetInWorldFrame_;
  RotationQuaternion desiredOrientationOffset_;

  /*! Load torso parameters from parameter file
   * @param hTorsoConfiguration A handle to the 'TorsoConfiguration' section in the parameter file
   * @returns true if successful
   */
  virtual bool loadParametersTorsoConfiguration(const TiXmlHandle& hTorsoConfiguration);

  /*! Load hip parameters from parameter file
   * @param hParameterSet A handle to the 'HipConfiguration' section in the parameter file
   * @returns true if successful
   */
  virtual bool loadParametersHipConfiguration(const TiXmlHandle &hParameterSet);

  virtual bool loadHeightTrajectory(const TiXmlHandle &hTrajectory, rbf::PeriodicRBF1DC1& trajectory);
  virtual bool interpolateHeightTrajectory(rbf::PeriodicRBF1DC1& interpolatedTrajectory,
                                           const rbf::PeriodicRBF1DC1& trajectory1,
                                           const rbf::PeriodicRBF1DC1& trajectory2,
                                           double t);

  RotationQuaternion getOrientationHeadingToDesiredHeadingBasedOnFeetLocations(const Position& positionWorldToDesiredHorizontalBaseInWorldFrame) const;
  RotationQuaternion getOrientationWorldToHeadingOnTerrainSurface(const RotationQuaternion& orientationWorldToHeading) const;
  RotationQuaternion getOrientationWorldToHeadingBasedOnHipLocations() const;

  virtual RotationQuaternion computeHeading(const RotationQuaternion& rquat, const Vector& axis);
  virtual RotationQuaternion decomposeRotation(const RotationQuaternion& q_BA, const Vector& vB);

};

} /* namespace loco */


#endif /* LOCO_TORSOCONTROLGAITCONTAINER_HPP_ */
