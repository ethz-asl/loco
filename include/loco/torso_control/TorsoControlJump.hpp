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

#include "GaussianKernelJumpPropagator.hpp"
#include <stdio.h>
#include <Eigen/Core>
#include <memory>

namespace loco {

enum Leg {
  FRONT_LEFT = 0,
  FRONT_RIGHT,
  HIND_RIGHT,
  HIND_LEFT,
};

enum State {
  INIT = 0,
  LIFTOFF,
  APEX,
  TOUCHDOWN,
  NUMBER_OF_STATES,
};

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

  void setInTorsoPositionMode (bool isInTorsoPositionMode);
  void setTrajectoryFollower(GaussianKernelJumpPropagator trajectoryFollower);
  void addMeasuresToTrajectory(double baseHeight);
  std::vector<double> getMeasuredTrajectory();
  std::vector<bool> getMeasuredContactFlags(Leg leg);

 private:
  double currentTime = 0;
  LegGroup* legs_;
  TorsoBase* torso_;
  loco::TerrainModelBase* terrain_;
  CoMOverSupportPolygonControl comControl_;
  GaussianKernelJumpPropagator trajectoryFollower_;
  State state_;
  bool inTorsoPositionMode_;

  void updateState();
  double headingDistanceFromForeToHindInBaseFrame_;
  std::ofstream output_;

  std::vector<double> measuredHeightTrajectory_;
  std::vector<bool> leftFrontContactFlagTrajectory_;
  std::vector<bool> rightFrontContactFlagTrajectory_;
  std::vector<bool> leftHindContactFlagTrajectory_;
  std::vector<bool> rightHindContactFlagTrajectory_;
};

}
/* namespace loco */
