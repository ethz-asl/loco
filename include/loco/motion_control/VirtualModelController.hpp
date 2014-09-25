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
  virtual bool loadParameters(const TiXmlHandle& handle);

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

  const Eigen::Vector3d& getProportionalGainTranslation() const;
  const Eigen::Vector3d& getDerivativeGainTranslation() const;
  const Eigen::Vector3d& getFeedforwardGainTranslation() const;

  const Eigen::Vector3d& getProportionalGainRotation() const;
  const Eigen::Vector3d& getDerivativeGainRotation() const;
  const Eigen::Vector3d& getFeedforwardGainRotation() const;

  void setProportionalGainTranslation(const Eigen::Vector3d& gains);
  void setDerivativeGainTranslation(const Eigen::Vector3d& gains);
  void setFeedforwardGainTranslation(const Eigen::Vector3d& gains);

  void setProportionalGainRotation(const Eigen::Vector3d& gains);
  void setDerivativeGainRotation(const Eigen::Vector3d& gains);
  void setFeedforwardGainRotation(const Eigen::Vector3d& gains);

  void setGainsHeading(double kp, double kd, double kff);
  void setGainsLateral(double kp, double kd, double kff);
  void setGainsVertical(double kp, double kd, double kff);
  void setGainsRoll(double kp, double kd, double kff);
  void setGainsPitch(double kp, double kd, double kff);
  void setGainsYaw(double kp, double kd, double kff);

  void getGainsHeading(double& kp, double& kd, double& kff);
  void getGainsLateral(double& kp, double& kd, double& kff);
  void getGainsVertical(double& kp, double& kd, double& kff);
  void getGainsRoll(double& kp, double& kd, double& kff);
  void getGainsPitch(double& kp, double& kd, double& kff);
  void getGainsYaw(double& kp, double& kd, double& kff);

  double getGravityCompensationForcePercentage() const;
  void setGravityCompensationForcePercentage(double percentage);
  const ContactForceDistributionBase& getContactForceDistribution() const;

  /*! Sets the parameters to the interpolated ones between motionController1 and controller2.
   * @param motionController1     If the interpolation parameter is 0, then the parameter set is equal to the one of motionController1.
   * @param motionController2     If the interpolation parameter is 1, then the parameter set is equal to the one of motionController2.
   * @param t                     interpolation parameter in [0, 1]
   * @return                      true if successful
   */
  virtual bool setToInterpolated(const MotionControllerBase& motionController1, const MotionControllerBase& motionController2, double t);

 private:
  std::shared_ptr<ContactForceDistributionBase> contactForceDistribution_;

  //! Base position error in base frame.
  Position positionErrorInControlFrame_;
  //! Base orientation error vector (rotation vector) in base frame.
  Eigen::Vector3d orientationError_;
  //! Base linear velocity error in base frame.
  LinearVelocity linearVelocityErrorInControlFrame_;
  //! Base angular velocity error in base frame.
  LocalAngularVelocity angularVelocityErrorInControlFrame_;

  //! Force on torso to compensate for gravity (in base frame).
  Force gravityCompensationForceInBaseFrame_;
  //! Torque on torso to compensate for gravity (in base frame).
  Torque gravityCompensationTorqueInBaseFrame_;

  //! Proportional gain vector (k_p)  for translational error (force).
  Eigen::Vector3d proportionalGainTranslation_;
  //! Derivative gain vector (k_d) for translational error (force).
  Eigen::Vector3d derivativeGainTranslation_;
  //! Feedforward gain vector (k_ff) for translational error (force).
  Eigen::Vector3d feedforwardGainTranslation_;

  //! Proportional (k_p), derivative (k_d), and feedforward gain vector (k_ff) for rotational error (torque).
  Eigen::Vector3d proportionalGainRotation_;
  Eigen::Vector3d derivativeGainRotation_;
  Eigen::Vector3d feedforwardGainRotation_;

  double gravityCompensationForcePercentage_;

  //! Desired virtual force on base in base frame (B_F_B^d).
  Force virtualForceInBaseFrame_;
  //! Desired virtual torque on base in base frame (B_T_B^d).
  Torque virtualTorqueInBaseFrame_;

 private:
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
