/*
 * RoughTerrain_Task.hpp
 *
 *  Created on: Jul 9, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef ROUGHTERRAIN_HPP_
#define ROUGHTERRAIN_HPP_

// robotTask
#include "TaskRobotBase.hpp"

// robotController
#include "VirtualModelController.hpp"
#include "ContactForceDistribution.hpp"

// robotUtils
#include "TaskTimer.hpp"
#include "DisturbRobot.hpp"

namespace robotTask {

//! Rough Terrain Task.
/*! Walk over rough terrain
 * @ingroup robotTask
 */
class RoughTerrain : public robotTask::TaskRobotBase {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! Constructor.
   * @param robotModel	reference to the robot
   */
  RoughTerrain(robotModel::RobotModel* robotModel);

  //! Destructor.
  ~RoughTerrain();

  /*! Adds the task.
   * @return	true if successful
   */
  virtual bool add();

  /*! Initializes the task.
   * @return	true if successful
   */
  virtual bool init();

  /*! Runs the task.
   * @return	true if successful
   */
  virtual bool run();

  /*! Changes the parameters of the task.
   * Loads a menu where the user can change parameters of this task.
   * @return	true if successful
   */
  virtual bool change();

 private:

  // Robot disturbances for testing
  robotUtils::DisturbRobot* disturbRobot_;

  //! Virtual model controller
  robotController::VirtualModelController* virtualModelController_;

};

}

#endif /* ROUGHTERRAIN_HPP_ */
