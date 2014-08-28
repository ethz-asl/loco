/*
 * MissionControlBase.cpp
 *
 *  Created on: Mar 7, 2014
 *      Author: gech
 */

#include "loco/mission_control/MissionControlBase.hpp"

namespace loco {

MissionControlBase::MissionControlBase() {


}

MissionControlBase::~MissionControlBase() {

}

bool MissionControlBase::setToInterpolated(const MissionControlBase& missionController1, const MissionControlBase& missionController2, double t) {
  return false;
}

} /* namespace loco */
