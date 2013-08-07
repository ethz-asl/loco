/*
 * VirtualModelController.hpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 * Description: Controller to stabilize the base of the robot based
 *              on a virtual spring-damper system. Based on 'Control
 *              of Dynamic Gaits for a Quadrupedal Robot',
 *              C. Gehring, ICRA, 2013.
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

  //! Load parameters.
  bool loadParameters();

  //! Compute joint torques for legs in stance.
  bool computeTorques(robotModel::VectorP desiredPosition,
                      Eigen::Quaterniond desiredOrientation,
                      robotModel::VectorP desiredLinearVelocity,
                      robotModel::VectorO desiredAngularVelocity);

 private:
  //! True when parameters are successfully loaded.
  bool isParametersLoaded;

  //! Contact force distribution.
  robotController::ContactForceDistribution* contactForceDistribution_;

  //! Base position error in base frame.
  robotModel::VectorP positionError_;
  //! Base orientation error vector in base frame (length of vector is angle).
  Eigen::Vector3d orientationErrorVector_;
  //! Base linear velocity error in base frame.
  robotModel::VectorP linearVelocityError_;
  //! Base angular velocity error in base frame.
  robotModel::VectorO angularVelocityError_;

  //! Proportional (k_p), derivative (k_d), and feedforward gain vector (k_ff) for translational error (force).
  Eigen::Vector3d proportionalGainTranslation_, derivativeGainTranslation_, feedforwardGainTranslation_;
  //! Proportional (k_p), derivative (k_d), and feedforward gain vector (k_ff) for rotational error (torque).
  Eigen::Vector3d proportionalGainRotation_, derivativeGainRotation_, feedforwardGainRotation_;
  //! Diagonal elements of the weighting matrix for the desired virtual forces and torques (S).
  Eigen::Vector3d virtualForceWeightningFactors_;
  //! Diagonal elements of the weighting matrix for the ground reaction forces ("regularizer", W).
  Eigen::Vector3d groundForceWeightningFactors_;
  //! Minimal normal ground force (F_min^n).
  double minimalNormalGroundForce_;
  //! Assumed friction coefficient (mu).
  double frictionCoefficient_;

  //! Desired virtual force on base in body frame (B_F_B^d).
  Eigen::Vector3d desiredVirtualForce_;
  //! Desired virtual torque on base in body frame (B_T_B^d).
  Eigen::Vector3d desiredVirtualTorque_;
  //! Effective virtual force on base in body frame (B_F_B).
  Eigen::Vector3d effectiveVirtualForce_;
  //! Effective virtual torque on base in body frame (B_T_B).
  Eigen::Vector3d effectiveVirtualTorque_;

  //! Compute error between desired an actual robot pose.
  bool computePoseError(robotModel::VectorP desiredPosition,
                        Eigen::Quaterniond desiredOrientation,
                        robotModel::VectorP desiredLinearVelocity,
                        robotModel::VectorO desiredAngularVelocity);

  //! Compute virtual force based on the form:
  //! F = k_p*(q_d-q) + k_d*(q_dot_d-q_dot) + k_ff*(...).
  bool computeVirtualForce(robotModel::VectorP desiredLinearVelocity);
  //! Compute virtual torque based on the form:
  //! T = k_p*(q_d-q) + k_d*(q_dot_d-q_dot) + k_ff*(...).
  bool computeVirtualTorque(robotModel::VectorO desiredAngularVelocity);

  //! Distributes the virtual forces to the ground contact forces.
  bool computeContactForces();

  //! Check if parameters loaded. True if parameters are successfully loaded.
  bool checkIfParametersLoaded();
};

} /* namespace robotController */
#endif /* VIRTUALMODELCONTROLLER_H_ */
