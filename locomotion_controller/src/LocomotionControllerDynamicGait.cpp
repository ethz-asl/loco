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

LocomotionControllerDynamicGait::LocomotionControllerDynamicGait(robotModel::RobotModel* robotModel, LimbCoordinatorBase* limbCoordinator, FootPlacementStrategyBase* footPlacementStrategy, BaseControlBase* baseController) :
    LocomotionControllerBase(),
    robotModel_(robotModel),
    limbCoordinator_(limbCoordinator),
    footPlacementStrategy_(footPlacementStrategy),
    baseController_(baseController)
{


}

LocomotionControllerDynamicGait::~LocomotionControllerDynamicGait() {

}

void LocomotionControllerDynamicGait::advance(double dt) {

  Eigen::Vector4i contactFlags = robotModel_->contacts().getCA();
  for (int iLeg=0; iLeg<4; iLeg++) {
    if (contactFlags(iLeg) == 1) {
      limbCoordinator_->setIsLegGrounded(iLeg, true);
    } else {
      limbCoordinator_->setIsLegGrounded(iLeg, false);
    }
  }
  limbCoordinator_->advance(dt);
//  FootPlacementStrategyDynamicGait* footPlacementStrategy =  (FootPlacementStrategyDynamicGait*)footPlacementStrategy_;


}

} /* namespace loco */
