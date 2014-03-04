/*
 * StateDynamicGait.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_STATEDYNAMICGAIT_HPP_
#define LOCO_STATEDYNAMICGAIT_HPP_


#include "loco/common/TorsoBase.hpp"


#include "kindr/poses/PoseDiffEigen.hpp"
#include "kindr/poses/PoseEigen.hpp"
#include <Eigen/Core>

#include "RobotModel.hpp"

namespace loco {


class TorsoStarlETH: public TorsoBase {
 public:
  TorsoStarlETH(robotModel::RobotModel* robotModel);
  virtual ~TorsoStarlETH();

  virtual double getStridePhase();
  virtual void setStridePhase(double stridePhase);


  virtual void advance(double dt);

  virtual TorsoStateMeasured& getMeasuredState();
  virtual TorsoStateDesired& getDesiredState();
protected:
  robotModel::RobotModel* robotModel_;

  TorsoStateMeasured stateMeasured_;
  TorsoStateDesired stateDesired_;

  double stridePhase_;








};

} /* namespace loco */

#endif /* LOCO_STATEDYNAMICGAIT_HPP_ */
