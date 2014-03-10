/*
 * TorsoPropertiesStarlETH.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/common/TorsoPropertiesStarlETH.hpp"
#include "loco/common/TypeDefs.hpp"

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
  setGravity(LinearAcceleration(0.0, 0.0, robotModel_->params().gravity_));
  setMass(robotModel_->params().mainbody_.m);
  setBaseToCenterOfMassPositionInBaseFrame(Position(0.0, 0.0, robotModel_->params().mainbody_.s));
  return true;
}

} /* namespace loco */
