/*!
* @file     LocomotionControllerDynamicGait.cpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#include "LocomotionControllerDynamicGait.hpp"

namespace loco {

LocomotionControllerDynamicGait::LocomotionControllerDynamicGait(LimbCoordinatorBase* limbCoordinator, FootPlacementStrategyBase* footPlacementStrategy, BaseControlBase* baseController) :
    LocomotionControllerBase(),
    limbCoordinator_(limbCoordinator),
    footPlacementStrategy_(footPlacementStrategy),
    baseController_(baseController)
{


}

LocomotionControllerDynamicGait::~LocomotionControllerDynamicGait() {

}

void LocomotionControllerDynamicGait::advance(double dt) {
  limbCoordinator_->advance(dt);

}

} /* namespace loco */
