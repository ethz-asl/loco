/*
 * TorsoStarlETH.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef LOCO_STATEDYNAMICGAIT_HPP_
#define LOCO_STATEDYNAMICGAIT_HPP_


#include "loco/common/TorsoBase.hpp"
#include "loco/common/TorsoPropertiesStarlETH.hpp"

#include "kindr/poses/PoseDiffEigen.hpp"
#include "kindr/poses/PoseEigen.hpp"
#include <Eigen/Core>

#include "RobotModel.hpp"

namespace loco {

//! Torso of StarlETH
/*! This should be used only as a data container
 *
 */
class TorsoStarlETH: public TorsoBase {
 public:
  TorsoStarlETH(robotModel::RobotModel* robotModel);
  virtual ~TorsoStarlETH();

  virtual double getStridePhase();
  virtual void setStridePhase(double stridePhase);

  virtual bool initialize(double dt);
  virtual bool advance(double dt);


  virtual TorsoStateMeasured& getMeasuredState();
  virtual TorsoStateDesired& getDesiredState();
  virtual const TorsoStateDesired& getDesiredState() const;

  virtual TorsoPropertiesBase& getProperties();

  friend std::ostream& operator << (std::ostream& out, const TorsoStarlETH& torso);
protected:
  robotModel::RobotModel* robotModel_;

  TorsoStateMeasured stateMeasured_;
  TorsoStateDesired stateDesired_;
  TorsoPropertiesStarlETH properties_;
  double stridePhase_;



};

} /* namespace loco */

#endif /* LOCO_STATEDYNAMICGAIT_HPP_ */
