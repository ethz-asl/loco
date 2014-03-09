/*
 * VirtualModelController.hpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef VIRTUALMODELCONTROLLER_H_
#define VIRTUALMODELCONTROLLER_H_

// Motion Controller
#include "loco/motion_control/MotionControllerBase.hpp"
// Contact force distribution
#include "loco/contact_force_distribution/ContactForceDistributionBase.hpp"
// Locomotion controller commons
#include "loco/common/TypeDefs.hpp"
// Eigen
#include <Eigen/Core>
// Cout
#include <iostream>

#include "tinyxml.h"

namespace loco {

class VirtualModelController : public MotionControllerBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Constructor.
   * @param robotModel the reference to the robot
   */
  VirtualModelController(std::shared_ptr<LegGroup> legs, std::shared_ptr<TorsoBase> torso,
                         std::shared_ptr<ContactForceDistributionBase> contactForceDistribution);
  /*!
   * Destructor.
   */
  virtual ~VirtualModelController();

  /*!
   * Load parameters.
   * @return true if successful
   */
  bool loadParameters();


  bool loadParameters(const TiXmlHandle& handle);

  /*!
   * Add data to logger (optional).
   * @return true if successful.
   */
  bool addToLogger();

  /*!
   * Computes the joint torques from the desired base pose.
   * @return true if successful.
   */
  bool compute();


  friend std::ostream& operator << (std::ostream& out, const VirtualModelController& motionController);


  Force getDesiredVirtualForceInBaseFrame() const;
  Torque getDesiredVirtualTorqueInBaseFrame() const;
  void getDistributedVirtualForceAndTorqueInBaseFrame(Force& netForce, Torque& netTorque) const;



 private:
  std::shared_ptr<ContactForceDistributionBase> contactForceDistribution_;

  //! Base position error in base frame.
  Position positionError_;
  //! Base orientation error vector (rotation vector) in base frame.
  Eigen::Vector3d orientationError_;
  //! Base linear velocity error in base frame.
  LinearVelocity linearVelocityError_;
  //! Base angular velocity error in base frame.
  LocalAngularVelocity angularVelocityError_;

  //! Force on torso to compensate for gravity (in base frame).
  Force gravityCompensationForce_;
  //! Torque on torso to compensate for gravity (in base frame).
  Torque gravityCompensationTorque_;

  //! Proportional (k_p), derivative (k_d), and feedforward gain vector (k_ff) for translational error (force).
  Eigen::Vector3d proportionalGainTranslation_, derivativeGainTranslation_, feedforwardGainTranslation_;
  //! Proportional (k_p), derivative (k_d), and feedforward gain vector (k_ff) for rotational error (torque).
  Eigen::Vector3d proportionalGainRotation_, derivativeGainRotation_, feedforwardGainRotation_;

  //! Desired virtual force on base in base frame (B_F_B^d).
  Force virtualForce_;
  //! Desired virtual torque on base in base frame (B_T_B^d).
  Torque virtualTorque_;

  /*!
   * Compute error between desired an actual robot pose and twist.
   * @return true if successful.
   */
  bool computeError();

  /*!
   * Computes the gravity compensation force and torque.
   * @return true if successful.
   */
  bool computeGravityCompensation();

  //! Compute virtual force based on the form:
  //! F = k_p*(q_d-q) + k_d*(q_dot_d-q_dot) + k_ff*(...).
  //! In base frame.
  bool computeVirtualForce();

  //! Compute virtual torque based on the form:
  //! T = k_p*(q_d-q) + k_d*(q_dot_d-q_dot) + k_ff*(...).
  //! In base frame.
  bool computeVirtualTorque();

  /*!
   * Check if parameters are loaded.
   * @return true if parameters are loaded
   */
  bool isParametersLoaded() const;
};

} /* namespace loco */
#endif /* VIRTUALMODELCONTROLLER_H_ */
