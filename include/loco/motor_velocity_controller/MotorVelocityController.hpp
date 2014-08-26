/*
 * MotorVelocityController.hpp
 *
 *  Created on: Jul 14, 2014
 *      Author: labstudent
 */

#ifndef MOTORVELOCITYCONTROLLER_HPP_
#define MOTORVELOCITYCONTROLLER_HPP_

#include "MotorVelocityControllerBase.hpp"
#include "loco/torso_control/TorsoControlJump.hpp"

#include "GaussianKernelJumpPropagator.hpp"
#include "loco/common/LegGroup.hpp"
#include "RobotModel.hpp"
#include <stdio.h>
#include <Eigen/Core>

namespace loco {

class MotorVelocityController : public MotorVelocityControllerBase {
 public:
  MotorVelocityController(robotModel::RobotModel* robotModel_, LegGroup* legs, TorsoBase* torso);
  virtual ~MotorVelocityController();
  virtual bool initialize(double dt);
  virtual void advance(double dt);

  virtual bool loadParameters(const TiXmlHandle& handle);

  void setInVelocityMode (bool isInVelocityMode);
  void setTrajectoryFollower(GaussianKernelJumpPropagator trajectoryFollower);
  State getState();

 private:
  bool inVelocityMode_;
  LegBase::JointPositions leftForeInitJointPositions_;
  LegGroup* legs_;
  TorsoBase* torso_;
  robotModel::RobotModel *robotModel_;
  GaussianKernelJumpPropagator trajectoryFollower_;
  State state_;

  void updateState();
};

}
/* namespace loco */


#endif /* MOTORVELOCITYCONTROLLER_HPP_ */
