/*
 * VirtualModelController.hpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef VIRTUALMODELCONTROLLER_H_
#define VIRTUALMODELCONTROLLER_H_

// Eigen
#include <Eigen/Geometry>

// robotTask
#include "ControllerBase.hpp"
#include "ContactForceDistribution.hpp"

namespace robotController {

class VirtualModelController : public robotController::ControllerBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! Constructor.
   * @param robotModel  reference to the robot
   */
  VirtualModelController(robotModel::RobotModel* robotModel);

  //! Destructor.
  virtual ~VirtualModelController();

  //! Compute joint torques for legs in stance
  bool computeTorques(robotModel::VectorP baseDesiredPosition,
                      Eigen::Quaterniond baseDesiredOrientation,
                      robotModel::VectorP baseDesiredLinearVelocity,
                      robotModel::VectorO baseDesiredAngularVelocity);

 private:
  //! Contact force distribution
  robotController::ContactForceDistribution* contactForceDistribution_;

  //! Desired base position expressed in inertial frame
  robotModel::VectorP desiredPosition_;
  //! Desired base orientation (Quaternion) w.r.t. inertial frame
  Eigen::Quaterniond desiredOrientation_;
  //! Desired base linear velocity expressed in inertial frame
  robotModel::VectorP desiredVelocity_;
  //! Desired base angular velocity expressed w.r.t. inertial frame
  robotModel::VectorO desiredAngularVelocity_;

  //! Base position error
  robotModel::VectorP positionError_;
  //! Base orientation error
  Eigen::Quaterniond orientationError_;
  //! Base linear velocity error
  robotModel::VectorP velocityError_;
  //! Base angular velocity error
  robotModel::VectorO angularVelocityError_;

  //! Proportional (k_p), derivative (k_d), and feedforward gain vector (k_ff)
  Eigen::VectorXd proportionalGain_, derivativeGain_, feedforwardGain_;
  //! Diagonal elements of the weighting matrix for the desired virtual forces and torques (S)
  Eigen::VectorXd virtualForceWeightningFactors_;
  //! Diagonal elements of the weighting matrix for the ground reaction forces ("regularizer", W)
  Eigen::VectorXd groundForceWeightningFactors_;
  //! Minimal normal ground force (F_min^n)
  double minimalNormalGroundForce_;
  //! Assumed friction coefficient (mu)
  double frictionCoefficient_;

  //! Desired virtual force on base in body frame (B_F_B^d)
  Eigen::Vector3d desiredVirtualForce_;
  //! Desired virtual torque on base in body frame (B_T_B^d)
  Eigen::Vector3d desiredVirtualTorque_;
  //! Effective virtual force on base in body frame (B_F_B)
  Eigen::Vector3d effectiveVirtualForce_;
  //! Effective virtual torque on base in body frame (B_T_B)
  Eigen::Vector3d effectiveVirtualTorque_;

  //! Compute error between desired an actual robot pose
  bool computePoseError();

  //! Compute virtual force and torque
  bool computeVirtualForce();

  //! Distributes the virtual forces to the ground contact forces
  bool computeContactForces();
};

} /* namespace robotController */
#endif /* VIRTUALMODELCONTROLLER_H_ */
