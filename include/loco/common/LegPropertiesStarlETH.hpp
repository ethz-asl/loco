/*
 * LegPropertiesStarlETH.hpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef LOCO_LEGPROPERTIESSTARLETH_HPP_
#define LOCO_LEGPROPERTIESSTARLETH_HPP_

#include "loco/common/LegPropertiesBase.hpp"

#include "RobotModel.hpp"

namespace loco {

class LegPropertiesStarlETH: public LegPropertiesBase {
 public:
  LegPropertiesStarlETH(int iLeg, robotModel::RobotModel* robotModel);
  virtual ~LegPropertiesStarlETH();
  virtual bool initialize(double dt);
  virtual bool advance(double dt);

 protected:
  int iLeg_;
  robotModel::RobotModel* robotModel_;
};

} /* namespace loco */

#endif /* LOCO_LEGPROPERTIESSTARLETH_HPP_ */
