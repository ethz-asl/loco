/*
 * RoughTerrain_Task.hpp
 *
 *  Created on: Jul 9, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef RoughTerrain_HPP_
#define RoughTerrain_HPP_

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

  robotUtils::DisturbRobot* disturbRobot_;

  //! Virtual model controller
  robotController::VirtualModelController* virtualModelController_;

  //! Contact force distribution
  robotController::ContactForceDistribution* contactForceDistribution_;

  //! Compute virtual force and torque on main body
  void computeVirtualModelControl();

  //! Compute virtual force and torque on main body
  void computeContactForceDistribution();

  // Task parameters
  //! Relative desired height of main body
  double height_;
  //! Relative desired yaw of main body
  double yaw_;
  //! Speed of base
  double baseMaxSpeed_;

  // Target and Desired main body variables
  //! Target vector from inertial frame to body frame expressed in inertial frame
  Eigen::Vector3d bodyTargetPosition_;
  //! Desired vector from inertial frame to body frame expressed in inertial frame
  Eigen::Vector3d bodyDesiredPosition_;
  //! Desired velocity of body frame expressed in inertial frame
  Eigen::Vector3d bodyDesiredVelocity_;
  //! Desired orientation of body frame w.r.t. inertial frame (Euler angles)
  Eigen::Vector3d bodyDesiredOrientation_;
  //! Desired rotational rate of body frame w.r.t. inertial frame
  Eigen::Vector3d bodyDesiredRotationalRate_;
};

}

#endif /* RoughTerrain_HPP_ */
