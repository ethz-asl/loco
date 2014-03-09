/*
 * TorsoPropertiesStarlETH.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/common/TorsoPropertiesStarlETH.hpp"

namespace loco {

TorsoPropertiesStarlETH::TorsoPropertiesStarlETH(robotModel::RobotModel* robotModel)
    : robotModel_(robotModel)
{

}

TorsoPropertiesStarlETH::~TorsoPropertiesStarlETH()
{

}

bool TorsoPropertiesStarlETH::update()
{
  setMass(robotModel_->params().mainbody_.m);
  setCenterOfMassInBaseFrame(Position(0.0, 0.0, robotModel_->params().mainbody_.s));
  return true;
}

} /* namespace loco */
