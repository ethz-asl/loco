/*!
* @file     BaseControlBase.cpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#include "loco/torso_control/TorsoControlBase.hpp"

namespace loco {

TorsoControlBase::TorsoControlBase() {


}

TorsoControlBase::~TorsoControlBase() {

}

bool TorsoControlBase::setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t) {
  return false;
}


} /* namespace loco */
