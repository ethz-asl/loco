/*!
 * @file     BaseControlJump.hpp
 * @author   Christian Gehring
 * @date     Feb, 2014
 * @version  1.0
 * @ingroup
 * @brief
 */
#ifndef LOCO_BASECONTROLDYNAMICGAIT_HPP_
#define LOCO_BASECONTROLDYNAMICGAIT_HPP_

#include "loco/torso_control/TorsoControlBase.hpp"
#include "loco/torso_control/CoMOverSupportPolygonControl.hpp"

#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"

#include "kindr/rotations/RotationEigen.hpp"

#include "loco/common/TerrainModelBase.hpp"

#include "GaussianKernel.hpp"

namespace loco {

class TorsoControlJump : public TorsoControlBase {
 public:
  TorsoControlJump(LegGroup* legs, TorsoBase* torso,
                   loco::TerrainModelBase* terrain);
  virtual ~TorsoControlJump();

  virtual bool initialize(double dt);
  virtual void advance(double dt);

  virtual RotationQuaternion computeHeading(const RotationQuaternion& rquat,
                                            const Vector& axis);
  RotationQuaternion decomposeRotation(const RotationQuaternion& q_BA,
                                       const Vector& vB);
  CoMOverSupportPolygonControl* getCoMControl();
  virtual bool loadParameters(const TiXmlHandle& handle);

 private:
  LegGroup* legs_;
  TorsoBase* torso_;
  loco::TerrainModelBase* terrain_;
  CoMOverSupportPolygonControl comControl_;

  double headingDistanceFromForeToHindInBaseFrame_;
  double desiredTorsoForeHeightAboveGroundInWorldFrameOffset_;
  double desiredTorsoHindHeightAboveGroundInWorldFrameOffset_;

  dmp::GaussianKernel desiredTrajectory_;
  virtual bool loadTrajectory(const TiXmlHandle &hTrajectory,
                              dmp::GaussianKernel& trajectory);

};

} /* namespace loco */

#endif /* LOCO_BASECONTROLDYNAMICGAIT_HPP_ */
