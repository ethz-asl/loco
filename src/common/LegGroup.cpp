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

} /* namespace loco */
