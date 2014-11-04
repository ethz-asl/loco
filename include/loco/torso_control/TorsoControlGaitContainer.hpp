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
