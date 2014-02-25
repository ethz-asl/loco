/*
 * StateDynamicGait.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#include "loco/common/StateDynamicGait.hpp"

namespace loco {

StateDynamicGait::StateDynamicGait() {


}

StateDynamicGait::~StateDynamicGait() {

}

double StateDynamicGait::getHeadingSpeed() {
  return robotAngularVelocity_.x();
}

double StateDynamicGait::getTurningSpeed() {
  return robotAngularVelocity_.z();
}

double StateDynamicGait::getLateralSpeed() {
  return robotAngularVelocity_.y();
}


} /* namespace loco */
