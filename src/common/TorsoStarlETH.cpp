/*
 * StateDynamicGait.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#include "loco/common/TorsoStarlETH.hpp"

namespace loco {

TorsoStarlETH::TorsoStarlETH() {


}

TorsoStarlETH::~TorsoStarlETH() {

}

double TorsoStarlETH::getHeadingSpeed() {
  return measuredRobotVelocity_.getTranslationalVelocity().x();
}

double TorsoStarlETH::getTurningSpeed() {
  return measuredRobotVelocity_.getRotationalVelocity().z();
}

double TorsoStarlETH::getLateralSpeed() {
  return measuredRobotVelocity_.getTranslationalVelocity().y();
}

double TorsoStarlETH::getDesiredHeadingSpeed() {
  return desiredRobotVelocity_.getTranslationalVelocity().x();
}

double TorsoStarlETH::getDesiredTurningSpeed() {
  return desiredRobotVelocity_.getRotationalVelocity().z();
}

double TorsoStarlETH::getDesiredLateralSpeed() {
  return desiredRobotVelocity_.getTranslationalVelocity().y();
}


} /* namespace loco */
