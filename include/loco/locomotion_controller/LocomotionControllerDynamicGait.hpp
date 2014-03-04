/*!
* @file     LocomotionControllerDynamicGait.hpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#ifndef LOCO_LOCOMOTIONCONTROLLERDYNAMICGAIT_HPP_
#define LOCO_LOCOMOTIONCONTROLLERDYNAMICGAIT_HPP_

#include "loco/locomotion_controller/LocomotionControllerBase.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"
#include "loco/limb_coordinator/LimbCoordinatorBase.hpp"
#include "loco/torso_control/TorsoControlBase.hpp"
#include "loco/contact_force_distribution/ContactForceDistributionBase.hpp"
#include "loco/virtual_model_control/VirtualModelController.hpp"

#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"

#include "RobotModel.hpp"
#include "TerrainBase.hpp"

namespace loco {

class LocomotionControllerDynamicGait: public LocomotionControllerBase {
 public:
  LocomotionControllerDynamicGait(LegGroup* legs, TorsoBase* torso, robotModel::RobotModel* robotModel,
                                  robotTerrain::TerrainBase* terrain, LimbCoordinatorBase* limbCoordinator,
                                  FootPlacementStrategyBase* footPlacementStrategy, TorsoControlBase* baseController,
                                  VirtualModelController* virtualModelController, ContactForceDistributionBase* contactForceDistribution);
  virtual ~LocomotionControllerDynamicGait();

  /*!
   * Initializes locomotion controller
   * @param dt the time step [s]
   * @return true if successfull.
   */
  virtual bool initialize(double dt);

  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual void advance(double dt);

  virtual TorsoBase* getTorso();
  virtual LegGroup* getLegs();
 protected:
  LegGroup* legs_;
  TorsoBase* torso_;
  robotModel::RobotModel* robotModel_;
  robotTerrain::TerrainBase* terrain_;
  LimbCoordinatorBase* limbCoordinator_;
  FootPlacementStrategyBase* footPlacementStrategy_;
  TorsoControlBase* torsoController_;
  VirtualModelController* virtualModelController_;
  ContactForceDistributionBase* contactForceDistribution_;
};

} /* namespace loco */

#endif /* LOCO_LOCOMOTIONCONTROLLERDYNAMICGAIT_HPP_ */
