/*
 * LegStarlETH.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#include "loco/common/LegStarlETH.hpp"

namespace loco {

LegStarlETH::LegStarlETH(const std::string& name, int iLeg,  robotModel::RobotModel* robotModel) :
  LegBase(name),
  iLeg_(iLeg),
  robotModel_(robotModel)
{

}

LegStarlETH::~LegStarlETH() {

}

const LegStarlETH::Position& LegStarlETH::getFootPositionInWorldFrame() {
  footPositionInWorldFrame_ = robotModel_->kin().getJacobianTByLeg_World2Foot_CSw(iLeg_)->getPos();
  return footPositionInWorldFrame_;
}

} /* namespace loco */
