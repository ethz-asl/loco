/*
 * StateBase.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#include "loco/common/TorsoBase.hpp"

namespace loco {

TorsoBase::TorsoBase() {

}

TorsoBase::~TorsoBase() {

}
std::ostream& operator << (std::ostream& out, const TorsoBase& torso) {
//  out << "Desired speed: " << torso.getDesiredState().getBaseTwistInBaseFrame();
  return out;
}

} /* namespace loco */
