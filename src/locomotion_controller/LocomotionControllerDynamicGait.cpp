/*!
* @file     LocomotionControllerDynamicGait.cpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"

namespace loco {

LocomotionControllerDynamicGait::LocomotionControllerDynamicGait(LegGroup* legs, TorsoBase* torso, robotModel::RobotModel* robotModel, robotTerrain::TerrainBase* terrain, LimbCoordinatorBase* limbCoordinator, FootPlacementStrategyBase* footPlacementStrategy, BaseControlBase* baseController) :
    LocomotionControllerBase(),
    legs_(legs),
    torso_(torso),
    robotModel_(robotModel),
    terrain_(terrain),
    limbCoordinator_(limbCoordinator),
    footPlacementStrategy_(footPlacementStrategy),
    baseController_(baseController)
{


}

LocomotionControllerDynamicGait::~LocomotionControllerDynamicGait() {

}

void LocomotionControllerDynamicGait::advance(double dt) {

  Eigen::Vector4i contactFlags = robotModel_->contacts().getCA();
  int iLeg = 0;
  for (auto leg : *legs_) {
    if (contactFlags(iLeg) == 1) {
      leg->setIsGrounded(true);
    }
    else {
      leg->setIsGrounded(false);
    }
    iLeg++;
  }
  limbCoordinator_->advance(dt);

  iLeg = 0;
  for (auto leg : *legs_) {
    const double swingPhase = leg->getSwingPhase();
    if ((swingPhase >= 0 && swingPhase <= 0.5) && leg->isInStanceMode()) {
      // possible lift-off
      leg->getStateLiftOff()->setFootPositionInWorldFrame(robotModel_->kin().getJacobianTByLeg_Base2Foot_CSw(iLeg)->getPos());
      leg->getStateLiftOff()->setHipPositionInWorldFrame(robotModel_->kin().getJacobianTByLeg_World2Hip_CSw(iLeg)->getPos());
      iLeg++;
    }
  }
  footPlacementStrategy_->advance(dt);
  baseController_->advance(dt);

}

} /* namespace loco */
