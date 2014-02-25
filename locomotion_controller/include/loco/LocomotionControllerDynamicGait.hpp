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

#include "loco/LocomotionControllerBase.hpp"
#include "loco/FootPlacementStrategyBase.hpp"
#include "loco/LimbCoordinatorBase.hpp"
#include "loco/BaseControlBase.hpp"

#include "RobotModel.hpp"

namespace loco {

class LocomotionControllerDynamicGait: public LocomotionControllerBase {
 public:
  LocomotionControllerDynamicGait(robotModel::RobotModel* robotModel, LimbCoordinatorBase* limbCoordinator, FootPlacementStrategyBase* footPlacementStrategy, BaseControlBase* baseController);
  virtual ~LocomotionControllerDynamicGait();

  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual void advance(double dt);

 protected:
  robotModel::RobotModel* robotModel_;
  LimbCoordinatorBase* limbCoordinator_;
  FootPlacementStrategyBase* footPlacementStrategy_;
  BaseControlBase* baseController_;
};

} /* namespace loco */

#endif /* LOCO_LOCOMOTIONCONTROLLERDYNAMICGAIT_HPP_ */
