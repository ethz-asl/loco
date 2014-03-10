/*
 * LegGroup.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#include "loco/common/LegGroup.hpp"

namespace loco {

LegGroup::LegGroup() {


}

LegGroup::~LegGroup() {

}

LegGroup::LegGroup(LegBase* leftForeLeg, LegBase* rightForeLeg, LegBase* leftHindLeg, LegBase* rightHindLeg) {
  addLeg(leftForeLeg);
  addLeg(rightForeLeg);
  addLeg(leftHindLeg);
  addLeg(rightHindLeg);
}

LegBase* LegGroup::getLeftForeLeg() {
  return legs_[0];
}
LegBase* LegGroup::getRightForeLeg() {
  return legs_[1];
}
LegBase* LegGroup::getLeftHindLeg() {
  return legs_[2];
}
LegBase* LegGroup::getRightHindLeg() {
  return legs_[3];
}



} /* namespace loco */
