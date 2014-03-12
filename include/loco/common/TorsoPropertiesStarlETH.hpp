/*
 * TorsoPropertiesStarlETH.hpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef LOCO_TORSOPROPERTIESSTARLETH_HPP_
#define LOCO_TORSOPROPERTIESSTARLETH_HPP_

#include "loco/common/TorsoPropertiesBase.hpp"

#include "RobotModel.hpp"

namespace loco {

class TorsoPropertiesStarlETH: public TorsoPropertiesBase {
 public:
  TorsoPropertiesStarlETH(robotModel::RobotModel* robotModel);
  virtual ~TorsoPropertiesStarlETH();
  virtual bool initialize(double dt);
  virtual bool advance(double dt);

 protected:
  robotModel::RobotModel* robotModel_;
};

} /* namespace loco */

#endif /* LOCO_TORSOPROPERTIESSTARLETH_HPP_ */
