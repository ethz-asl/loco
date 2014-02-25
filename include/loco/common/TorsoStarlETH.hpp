/*
 * StateDynamicGait.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_STATEDYNAMICGAIT_HPP_
#define LOCO_STATEDYNAMICGAIT_HPP_


#include "loco/common/TorsoBase.hpp"
#include "kindr/positions/PositionDiffEigen.hpp"
#include "kindr/rotations/RotationDiffEigen.hpp"
#include <Eigen/Core>

namespace loco {


class TorsoStarlETH: public TorsoBase {
 public:
  typedef kindr::positions::eigen_impl::LinearVelocityD LinearVelocity;
  typedef kindr::rotations::eigen_impl::LocalAngularVelocityPD LocalAngularVelocity;
 public:
  TorsoStarlETH();
  virtual ~TorsoStarlETH();

  virtual double getHeadingSpeed();
  virtual double getTurningSpeed();
  virtual double getLateralSpeed();
 protected:
  LinearVelocity robotLinearVelocity_;
  LocalAngularVelocity robotAngularVelocity_;
};

} /* namespace loco */

#endif /* LOCO_STATEDYNAMICGAIT_HPP_ */
