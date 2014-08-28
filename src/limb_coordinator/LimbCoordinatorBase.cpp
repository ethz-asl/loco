/*!
* @file     LimbCoordinatorBase.cpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#include "loco/limb_coordinator/LimbCoordinatorBase.hpp"

namespace loco {

LimbCoordinatorBase::LimbCoordinatorBase() {


}

LimbCoordinatorBase::~LimbCoordinatorBase() {

}

bool LimbCoordinatorBase::setToInterpolated(const LimbCoordinatorBase& limbCoordinator1, const LimbCoordinatorBase& limbCoordinator2, double t) {
  return false;
}

} /* namespace loco */
