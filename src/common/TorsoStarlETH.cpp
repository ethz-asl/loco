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
  return robotAngularVelocity_.x();
}

double TorsoStarlETH::getTurningSpeed() {
  return robotAngularVelocity_.z();
}

double TorsoStarlETH::getLateralSpeed() {
  return robotAngularVelocity_.y();
}


} /* namespace loco */
