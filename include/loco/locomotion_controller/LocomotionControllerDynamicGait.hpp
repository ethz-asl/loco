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

#include "loco/contact_detection/ContactDetectorBase.hpp"
#include "loco/terrain_perception/TerrainPerceptionBase.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"
#include "loco/limb_coordinator/LimbCoordinatorBase.hpp"
#include "loco/torso_control/TorsoControlBase.hpp"
#include "loco/contact_force_distribution/ContactForceDistributionBase.hpp"
#include "loco/motion_control/VirtualModelController.hpp"

#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"
#include "loco/common/ParameterSet.hpp"


namespace loco {

class LocomotionControllerDynamicGait: public LocomotionControllerBase {
 public:
  LocomotionControllerDynamicGait(LegGroup* legs, TorsoBase* torso,
                                  TerrainPerceptionBase* terrainPerception,
                                  ContactDetectorBase* contactDetector,
                                  LimbCoordinatorBase* limbCoordinator,
                                  FootPlacementStrategyBase* footPlacementStrategy, TorsoControlBase* baseController,
                                  VirtualModelController* virtualModelController, ContactForceDistributionBase* contactForceDistribution,
                                  ParameterSet* parameterSet);
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
  virtual bool advance(double dt);


  virtual bool isInitialized() const;

  virtual TorsoBase* getTorso();
  virtual LegGroup* getLegs();

  FootPlacementStrategyBase* getFootPlacementStrategy();
  VirtualModelController* getVirtualModelController();
  ContactForceDistributionBase* getContactForceDistribution();
  LimbCoordinatorBase* getLimbCoordinator();
  TerrainPerceptionBase* getTerrainPerception();
 protected:
  bool isInitialized_;
  LegGroup* legs_;
  TorsoBase* torso_;
  TerrainPerceptionBase* terrainPerception_;
  ContactDetectorBase* contactDetector_;
  LimbCoordinatorBase* limbCoordinator_;
  FootPlacementStrategyBase* footPlacementStrategy_;
  TorsoControlBase* torsoController_;
  VirtualModelController* virtualModelController_;
  ContactForceDistributionBase* contactForceDistribution_;
  ParameterSet* parameterSet_;
};

} /* namespace loco */

#endif /* LOCO_LOCOMOTIONCONTROLLERDYNAMICGAIT_HPP_ */
