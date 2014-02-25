/*
 * ContactForceDistribution.hpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "ContactForceDistributionBase.hpp"
#include <Eigen/Sparse>

namespace loco {

//! This class distributes a virtual force and torque on the base as forces to the leg contact points.
/*!
 * Based on 'Control of Dynamic Gaits for a Quadrupedal Robot', C. Gehring, ICRA, 2013.
 *
 * The optimization problem is formulated as:
 *
 * [ I    I   ...] [f1] = [F] ==> A*x = b
 * [r1x  r2x  ...] [f2]   [T]
 *                 [ .]
 *                 [ .]
 *                 [ .]
 */
class ContactForceDistribution : public ContactForceDistributionBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
   * Loads the parameters. Has to be done before using this class.
   * @return true if successful.
   */
  bool loadParameters();

  /*!
   * Adds class data to the logger (optional).
   * @return true if successful.
   */
  bool addToLogger();

  /*!
   * (Optional) Change how much a leg should be loaded. This needs to be set before
   * calling computeForceDistribution() and is reset to 1 after each call of
   * computeForceDistribution(). The contact force distribution is calculated
   * twice, once without the load factor equality constraint and then
   * including the equality constraint.
   * @param leg defines the leg
   * @param loadFactor sets the factor how much the leg should be loaded
   *        (related to the unconstrained case without user specified load
   *        factors), value in the interval [0, 1] where 0: unloaded
   *        and 1: completely loaded.
   * @return true if successful, false if leg is not in contact with the
   *         ground (unless loadFactor = 0).
   */
  bool changeLegLoad(const Legs& leg, const double& loadFactor);


  /*!
   * Computes the contact force distribution of the virtual force and torque.
   * @param virtualForce the desired virtual force on the base (in base frame).
   * @param virtualTorque the desired virtual torque on the base (in base frame).
   * @return true if successful.
   */
  bool computeForceDistribution(const Eigen::Vector3d& virtualForce,
                                        const Eigen::Vector3d& virtualTorque);

  /*!
   * Gets the distributed force for a leg at the contact point.
   * @param[in] leg defines the leg.
   * @param[out] force is the distributed force on the leg (in base frame).
   * @return true if leg is active (in stance) and force can be applied, false otherwise.
   */
  bool getForceForLeg(const Legs& leg, robotModel::VectorCF& force);

  /*!
   * Gets the distributed net forces and torques that act on the base, i.e.
   * this force and torque are computed from the distributed contact forces and
   * should ideally be equal to the desired net forces and torques.
   * @param[out] netForce.
   * @param[out] netTorque.
   * @return true if net force and torque can be calculated, false otherwise.
   */
   bool getNetForceAndTorqueOnBase(Eigen::Vector3d& netForce, Eigen::Vector3d& netTorque);

 private:
  //! Number of legs in stance phase
  int nLegsInStance_;
  //! Number of variables to optimize (size of x, n = nTranslationalDofPerFoot_ * nLegsInStance_)
  int n_;

  //! Diagonal elements of the weighting matrix for the desired virtual forces and torques (for S).
  Eigen::Matrix<double, nElementsVirtualForceTorqueVector_, 1> virtualForceWeights_;
  //! Diagonal element of the weighting matrix for the ground reaction forces (regularizer, for W).
  double groundForceWeight_;
  //! Minimal normal ground force (F_min^n, in N).
  double minimalNormalGroundForce_;
  //! Assumed friction coefficient (mu).
  double frictionCoefficient_;

  //! Stacked contact forces (in base frame)
  Eigen::VectorXd x_;
  //! The matrix A in the optimization formulation (nElementsInStackedVirtualForceTorqueVector_ x n).
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_;
  //! The vector b in the optimization formulation (stacked vector of desired net virtual forces and torques).
  Eigen::VectorXd b_;
  //! Weighting matrix for the ground reaction forces (regularizer).
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> W_;
  //! Weighting matrix for the desired virtual forces and torques.
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> S_;
  //! Force inequality constraint matrix
  Eigen::SparseMatrix<double, Eigen::RowMajor> D_;
  //! Upper and lower limits vectors of force inequality constraint
  Eigen::VectorXd d_, f_;
  //! Force equality constraint matrix
  Eigen::SparseMatrix<double, Eigen::RowMajor> C_;
  //! Vector of force equality constraint
  Eigen::VectorXd c_;

  struct LegStatus
  {
    bool isInStance_;
    double loadFactor_;
    bool isLoadConstraintActive_;
    int indexInStanceLegList_;
    int startIndexInVectorX_;
    robotModel::VectorCF effectiveContactForce_; // in world frame
    Eigen::Vector3d terrainNormalInWorldFrame_;
    Eigen::Vector3d terrainNormalInBaseFrame_;
  };

  std::map<Legs, LegStatus> legStatuses_;

  /*!
   * Reads foot contact flags and includes user leg load settings from changeLegLoad().
   * @return true if successful.
   */
  bool prepareLegLoading();

  /*!
   * Prepare matrices for the optimization problem.
   * @return true if successful
   */
  bool prepareOptimization(const Eigen::Vector3d& virtualForce,
                           const Eigen::Vector3d& virtualTorque);

  bool setTerrainNormalsToDefault();

  bool getTerrainNormals();

  bool addMinimalForceConstraints();

  bool addFrictionConstraints();

  bool addDesiredLegLoadConstraints();

  /*!
   * Solve optimization
   * @return true if successful
   */
  bool solveOptimization();

  bool resetFootLoadFactors();

  bool resetConstraints();

  /*!
   * Update the data that is recorded by the logger.
   * @return true if successful, false if it is not logging.
   */
  bool updateLoggerData();
};

} /* namespace loco */
