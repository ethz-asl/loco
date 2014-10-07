/*!
* @file     BaseControlBase.hpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/

#ifndef LOCO_BASECONTROLBASE_HPP_
#define LOCO_BASECONTROLBASE_HPP_

#include "tinyxml.h"
#include "PeriodicRBF1DC1.hpp"
#include "loco/common/TypeDefs.hpp"
#include "kindr/rotations/RotationEigen.hpp"

namespace loco {

class TorsoControlBase {
 public:
  TorsoControlBase();
  virtual ~TorsoControlBase();
  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;
  virtual bool loadParameters(const TiXmlHandle& handle) = 0;

  virtual RotationQuaternion computeHeading(const RotationQuaternion& rquat, const Vector& axis);
  RotationQuaternion decomposeRotation(const RotationQuaternion& q_BA, const Vector& vB);

  /*! Computes an interpolated version of the two controllers passed in as parameters.
   *  if t is 0, the current setting is set to controller1, 1 -> controller2, and values in between
   *  correspond to interpolated parameter set.
   * @param torsoController1
   * @param torsoController2
   * @param t interpolation parameter
   * @returns true if successful
   */
  virtual bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t);

  double getDesiredTorsoForeHeightAboveGroundInWorldFrameOffset() const;
  double getDesiredTorsoHindHeightAboveGroundInWorldFrameOffset() const;

  void setDesiredPositionOffetInWorldFrame(const Position& positionOffsetInWorldFrame);
  void setDesiredOrientationOffset(const RotationQuaternion& orientationOffset);

 protected:
  double headingDistanceFromForeToHindInBaseFrame_;
  double desiredTorsoForeHeightAboveGroundInWorldFrameOffset_;
  double desiredTorsoHindHeightAboveGroundInWorldFrameOffset_;
  rbf::PeriodicRBF1DC1 desiredTorsoForeHeightAboveGroundInWorldFrame_;
  rbf::PeriodicRBF1DC1 desiredTorsoHindHeightAboveGroundInWorldFrame_;
  Position desiredPositionOffsetInWorldFrame_;
  RotationQuaternion desiredOrientationOffset_;

  virtual bool loadParametersHipConfiguration(const TiXmlHandle &hParameterSet);
  virtual bool loadHeightTrajectory(const TiXmlHandle &hTrajectory,  rbf::PeriodicRBF1DC1& trajectory);
  bool interpolateHeightTrajectory(rbf::PeriodicRBF1DC1& interpolatedTrajectory, const rbf::PeriodicRBF1DC1& trajectory1, const rbf::PeriodicRBF1DC1& trajectory2, double t);

};

} /* namespace loco */

#endif /* BASECONTROLBASE_HPP_ */
