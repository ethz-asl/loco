/*!
 * @file     BaseControlJump.hpp
 * @author   Christian Gehring
 * @date     Feb, 2014
 * @version  1.0
 * @ingroup
 * @brief
 */

#pragma once

#include "loco/torso_control/TorsoControlBase.hpp"
#include "loco/torso_control/CoMOverSupportPolygonControl.hpp"

#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"

#include "kindr/rotations/RotationEigen.hpp"

#include "loco/common/TerrainModelBase.hpp"

#include "GaussianKernel.hpp"
#include <stdio.h>
#include <Eigen/Core>

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

  void addMeasureToTrajectory(double baseHeight);
  std::vector<double> getMeasuredTrajectory();
  double getMaxDuration();
  double getProgress();

 private:
  LegGroup* legs_;
  TorsoBase* torso_;
  loco::TerrainModelBase* terrain_;
  CoMOverSupportPolygonControl comControl_;
  dmp::GaussianKernel desiredTrajectory_;

  double currentTime_;
  double maxDuration_;
  double headingDistanceFromForeToHindInBaseFrame_;

  std::vector<double> measuredHeightTrajectory_;
  Eigen::VectorXd *thetas_;

  virtual bool loadTrajectory(const TiXmlHandle &hJump);
  virtual bool loadGaussianKernel(const TiXmlHandle &hTrajectory);
  virtual bool loadMovement(const TiXmlHandle &hTrajectory);
  virtual bool loadThetas(const TiXmlHandle &hThetas);

  void resetTime();
  void incrementTime(double dt);
};

}
/* namespace loco */
