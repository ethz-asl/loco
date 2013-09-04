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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ControllerBase.hpp"
#include "ContactForceDistribution.hpp"
#include "toMove.hpp"

namespace robotController {

class VirtualModelController : public robotController::ControllerBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Constructor.
   * @param robotModel the reference to the robot
   */
  VirtualModelController(robotModel::RobotModel* robotModel);

  /*!
   * Destructor.
   */
  virtual ~VirtualModelController();

  /*!
   * Load parameters.
   * @return true if successful
   */
  bool loadParameters();

  /*!
   * Add data to logger (optional).
   * @return true if successful
   */
  bool addToLogger();

  /*!
   * Computes the joint torques from the desired base pose.
   * @param[in] desiredPosition in the world frame
   * @param[in] desiredOrientation w.r.t. the world frame
   * @param[in] desiredLinearVelocity in the world frame
   * @param[in] desiredAngularVelocity w.r.t. the world frame in the world frame
   * @return true if successful
   */
  bool computeTorques(const robotModel::VectorP& desiredPosition, // TODO pack as one struct?
                      const Eigen::Quaterniond& desiredOrientation,
                      const robotModel::VectorP& desiredLinearVelocity,
                      const robotModel::VectorO& desiredAngularVelocity);

  /*!
   * Gets access to the contactForceDistribution class.
   * @return pointer to the contactForceDistribution class
   */
  robotController::ContactForceDistribution* getContactForceDistributor() const;

  /*!
   * Prints information for debugging.
   */
  void printDebugInformation();

 private:
  //! True if parameters are successfully loaded.
  bool isParametersLoaded_;

  //! Contact force distribution.
  robotController::ContactForceDistribution* contactForceDistribution_;

  //! Base position error in base frame.
  robotModel::VectorP positionError_;
  //! Base orientation error vector in base frame.
  Eigen::AngleAxisd orientationError_;
  //! Base linear velocity error in base frame.
  robotModel::VectorP linearVelocityError_;
  //! Base angular velocity error in base frame.
  robotModel::VectorO angularVelocityError_;

  //! Proportional (k_p), derivative (k_d), and feedforward gain vector (k_ff) for translational error (force).
  Eigen::Vector3d proportionalGainTranslation_, derivativeGainTranslation_, feedforwardGainTranslation_;
  //! Proportional (k_p), derivative (k_d), and feedforward gain vector (k_ff) for rotational error (torque).
  Eigen::Vector3d proportionalGainRotation_, derivativeGainRotation_, feedforwardGainRotation_;

  //! Desired virtual force on base in base frame (B_F_B^d).
  Eigen::Vector3d virtualForce_;
  //! Desired virtual torque on base in base frame (B_T_B^d).
  Eigen::Vector3d virtualTorque_;

  //! Compute error between desired an actual robot pose.
  //! In world coordinate system
  bool computePoseError(const robotModel::VectorP& desiredPosition,
                        const Eigen::Quaterniond& desiredOrientation,
                        const robotModel::VectorP& desiredLinearVelocity,
                        const robotModel::VectorO& desiredAngularVelocity);

  //! Compute virtual force based on the form:
  //! F = k_p*(q_d-q) + k_d*(q_dot_d-q_dot) + k_ff*(...).
  //! In base frame.
  bool computeVirtualForce(const robotModel::VectorP& desiredLinearVelocity);

  //! Compute virtual torque based on the form:
  //! T = k_p*(q_d-q) + k_d*(q_dot_d-q_dot) + k_ff*(...).
  //! In base frame.
  bool computeVirtualTorque(const robotModel::VectorO& desiredAngularVelocity);

  //! Distributes the virtual forces to the ground contact forces.
  bool computeJointTorques();

  /*!
   * Calculate the joints torque from a desired contact force
   * with Jacobi transpose.
   * @param [in] leg
   * @param [in] legIndexInStackedVector
   * @param [in] contactForce
   * @param [out] jointTorque
   * @return true if successfull
   */
  bool computeJointTorquesForLeg(const Legs& leg,
                          const int& legIndexInStackedVector,
                          const robotModel::VectorCF& contactForce,
                          robotModel::VectorActLeg& jointTorque);

  /*!
   * Check if parameters are loaded.
   * @return true if parameters are loaded
   */
  bool isParametersLoaded() const;
};

} /* namespace robotController */
#endif /* VIRTUALMODELCONTROLLER_H_ */
