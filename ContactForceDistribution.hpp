/*
 * ContactForceDistribution.hpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 * Description: This class distributes a virtual force and torque
 *              as forces to the leg contact points. Based on
 *              'Control of Dynamic Gaits for a Quadrupedal Robot',
 *              C. Gehring, ICRA, 2013.
 *
 *
 * [ I    I   ...] [f1] = [F] ==> A*x = b
 * [r1x  r2x  ...] [f2]   [T]
 *                 [ .]
 *                 [ .]
 *                 [ .]
 */

#ifndef CONTACTFORCEDISTRIBUTION_HPP_
#define CONTACTFORCEDISTRIBUTION_HPP_

#include <Eigen/Core>
#include "RobotModel.hpp"

namespace robotController {

class ContactForceDistribution
{
 public:
  /*!
   * Constructor.
   * @param robotModel the reference to the robot
   */
  ContactForceDistribution(robotModel::RobotModel* robotModel);

  /*!
   * Destructor.
   */
  virtual ~ContactForceDistribution();

  /*!
   * Leg enum.
   * TODO move this to robot commons?
   */
  enum LegName {
    LEFT_FRONT = 0,
    RIGHT_FRONT,
    LEFT_HIND,
    RIGHT_HIND
  };

  /*!
   * Loads the parameters. Has to be done before using this class.
   * @return true if successful
   */
  bool loadParameters();

  /*!
   * Computes the contact force distribution of the virtual force and torque.
   * @param desiredVirtualForce
   * @param desiredVirtualTorque
   * @return true if successful
   */
  bool computeForceDistribution(const Eigen::Vector3d& virtualForce,
                                const Eigen::Vector3d& virtualTorque);

  /*!
   * Manually change how much a leg should be loaded.
   * This needs to be set before calling computeForceDistribution() and
   * is reset after each call of computeForceDistribution().
   * @param legName defines the leg
   * @param loadFactor sets the factor how much the leg should be loaded
   *        value in the interval [0, 1] where 0: unloaded and 1: completely loaded
   * @return true if successful, false if leg is not in contact with the ground (unless loadFactor=0.0)
   */
  bool changeLegLoad(const LegName& legName, const double& loadFactor);

 private:
  //! Defines. TODO change to 'constexpr'
  #define N_LEGS 4 // TODO move to robotCommons
  #define N_CONSTRAINTS_PER_FOOT 5
  #define N_TRANSLATIONAL_DOF_PER_FOOT 3

  //! Reference to robot model
  robotModel::RobotModel* robotModel_;

  //! True when parameters are successfully loaded.
  bool isParametersLoaded_;

  //! Leg load factors
  Eigen::Matrix<double, N_LEGS, 1> legLoadFactors_;
  //! Number of legs in stance phase
  int nLegsInStance_;

  //! Diagonal elements of the weighting matrix for the desired virtual forces and torques (S).
  Eigen::Matrix<double, 6, 1> virtualForceWeights_;
  //! Diagonal element of the weighting matrix for the ground reaction forces ("regularizer", W).
  double groundForceWeight_;
  //! Minimal normal ground force (F_min^n, in N).
  double minimalNormalGroundForce_;
  //! Assumed friction coefficient (mu).
  double frictionCoefficient_;

  //! The matrix A in the optimization formulation.
  Eigen::MatrixXd optimizationMatrixA_;
  //! The vector b in the optimization formulation (net forces and torques).
  Eigen::Matrix<double, 6, 1> stackedVirtualForceAndTorque_;
  //! Constraint matrix
//  Eigen::SparseMatrix optimizationConstraintMatrix_;
  //! Upper and lower limits vectors
  Eigen::VectorXd optimizationUpperLimits_, optimizationLowerLimits_;
  //! Stacked contact forces (x)
  Eigen::VectorXd stackedContactForces_;
  //! Weighting matrix for the ground reaction forces ("regularizer", W).
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> groundForceWeights_;
  //! Weighting matrix for the desired virtual forces and torques (S).
  Eigen::DiagonalMatrix<double, 6> virtualForceWeightsMatrix_;

  /*!
   * Check if parameters are loaded.
   * @return true if parameters are loaded.
   */
  bool areParametersLoaded();

  /*!
   * Reads foot contact flags and includes user leg load settings from changeLegLoad()
   * @return true if successful
   */
  bool prepareDesiredLegLoading();

  /*!
   * Prepare matrices for the optimization problem.
   * @return true if successful
   */
  bool prepareOptimization(const Eigen::Vector3d& virtualForce,
                           const Eigen::Vector3d& virtualTorque);


};

} /* namespace robotController */
#endif /* CONTACTFORCEDISTRIBUTION_HPP_ */
