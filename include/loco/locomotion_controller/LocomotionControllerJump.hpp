/*!
* @file     LocomotionControllerJump.hpp
* @author   wko
* @date     Jun, 2014
* @version  1.0
* @ingroup
* @brief
*/
#ifndef LOCO_LOCOMOTIONCONTROLLERJUMP_HPP_
#define LOCO_LOCOMOTIONCONTROLLERJUMP_HPP_

#include "loco/locomotion_controller/LocomotionControllerBase.hpp"

#include "loco/contact_detection/ContactDetectorBase.hpp"
#include "loco/terrain_perception/TerrainPerceptionBase.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"
#include "loco/torso_control/TorsoControlBase.hpp"
#include "loco/contact_force_distribution/ContactForceDistributionBase.hpp"
#include "loco/motion_control/VirtualModelController.hpp"
#include "loco/limb_coordinator/LimbCoordinatorBase.hpp"
#include "loco/event_detection/EventDetector.hpp"

#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"
#include "loco/common/ParameterSet.hpp"

#include "GaussianKernelJumpPropagator.hpp"

namespace loco {

class LocomotionControllerJump: public LocomotionControllerBase {
 public:
  LocomotionControllerJump(LegGroup* legs, TorsoBase* torso,
                                  TerrainPerceptionBase* terrainPerception,
                                  ContactDetectorBase* contactDetector,
                                  LimbCoordinatorBase* limbCoordinator,
                                  FootPlacementStrategyBase* footPlacementStrategy,
                                  TorsoControlBase* baseController,
                                  VirtualModelController* virtualModelController, ContactForceDistributionBase* contactForceDistribution,
                                  ParameterSet* parameterSet,
                                  TerrainModelBase* terrainModel);
  virtual ~LocomotionControllerJump();

  /*!
   * Initializes locomotion controller
   * @param dt the time step [s]
   * @return true if successful.
   */
  virtual bool initialize(double dt);

  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual bool advanceMeasurements(double dt);
  virtual bool advanceSetPoints(double dt);

  virtual double getRuntime() const;

  virtual bool isInitialized() const;

  virtual TorsoBase* getTorso();
  virtual LegGroup* getLegs();

  TorsoControlBase* getTorsoController();
  FootPlacementStrategyBase* getFootPlacementStrategy();
  VirtualModelController* getVirtualModelController();
  ContactForceDistributionBase* getContactForceDistribution();
  LimbCoordinatorBase* getLimbCoordinator();
  TerrainPerceptionBase* getTerrainPerception();

 protected:
  GaussianKernelJumpPropagator trajectoryFollower_;
  double runtime_;
  bool isInitialized_;
  LegGroup* legs_;
  TorsoBase* torso_;
  LimbCoordinatorBase* limbCoordinator_;
  TerrainPerceptionBase* terrainPerception_;
  ContactDetectorBase* contactDetector_;
  FootPlacementStrategyBase* footPlacementStrategy_;
  TorsoControlBase* torsoController_;
  VirtualModelController* virtualModelController_;
  ContactForceDistributionBase* contactForceDistribution_;
  ParameterSet* parameterSet_;
  EventDetectorBase* eventDetector_;
  TerrainModelBase* terrainModel_;
};

} /* namespace loco */

#endif /* LOCO_LOCOMOTIONCONTROLLERJUMP_HPP_ */
