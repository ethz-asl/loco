/*
 * LegPropertiesStarlETH.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/common/LegPropertiesStarlETH.hpp"

namespace loco {

LegPropertiesStarlETH::LegPropertiesStarlETH(int iLeg, robotModel::RobotModel* robotModel)
    : iLeg_(iLeg),
      robotModel_(robotModel)
{

}

LegPropertiesStarlETH::~LegPropertiesStarlETH() {

}

bool LegPropertiesStarlETH::update()
{
  double mass = robotModel_->params().hip_.m + robotModel_->params().thigh_.m + robotModel_->params().shank_.m;
  setMass(mass);

  Position positionBaseToCenterOfMassInBaseFrame = Position(
     (robotModel_->kin().getJacobianTByLeg_Base2HipCoG_CSmb(iLeg_)->getPos() * robotModel_->params().hip_.m
    + robotModel_->kin().getJacobianTByLeg_Base2ThighCoG_CSmb(iLeg_)->getPos() * robotModel_->params().thigh_.m
    + robotModel_->kin().getJacobianTByLeg_Base2ShankCoG_CSmb(iLeg_)->getPos() * robotModel_->params().shank_.m) / mass);
  setBaseToCenterOfMassPositionInBaseFrame(positionBaseToCenterOfMassInBaseFrame);

  return true;
}

} /* namespace loco */
