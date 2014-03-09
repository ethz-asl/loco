/*
 * ContactForceDistribution.hpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "ContactForceDistributionBase.hpp"
#include "loco/common/LegBase.hpp"
#include <Eigen/SparseCore>
#include "tinyxml.h"

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
   */
  ContactForceDistribution(std::shared_ptr<LegGroup> legs, std::shared_ptr<robotTerrain::TerrainBase> terrain);

  /*!
   * Destructor.
   */
  virtual ~ContactForceDistribution();

  /*!
   * Loads the parameters. Has to be done before using this class.
   * @return true if successful.
   */
  bool loadParameters(const TiXmlHandle& handle);

  /*!
   * Adds class data to the logger (optional).
   * @return true if successful.
   */
  bool addToLogger();

  /*!
   * Computes the contact force distribution of the virtual force and torque.
   * @param virtualForce the desired virtual force on the base (in base frame).
   * @param virtualTorque the desired virtual torque on the base (in base frame).
   * @return true if successful.
   */
  bool computeForceDistribution(const Force& virtualForce,
                                const Torque& virtualTorque);

  /*!
   * Gets the distributed net forces and torques that act on the base, i.e.
   * this force and torque are computed from the distributed contact forces and
   * should ideally be equal to the desired net forces and torques.
   * @param[out] netForce.
   * @param[out] netTorque.
   * @return true if net force and torque can be calculated, false otherwise.
   */
   bool getNetForceAndTorqueOnBase(Force& netForce, Torque& netTorque);

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

  struct LegInfo
  {
    bool isPartOfOptimization_;
    bool isLoadConstraintActive_;
    int indexInStanceLegList_;
    int startIndexInVectorX_;
    Force desiredContactForce_;
  };

  std::map<LegBase*, LegInfo> legInfos_;

  /*!
   * Reads foot contact flags and includes user leg load settings from changeLegLoad().
   * @return true if successful.
   */
  bool prepareLegLoading();

  /*!
   * Prepare matrices for the optimization problem.
   * @return true if successful
   */
  bool prepareOptimization(const Force& virtualForce,
                           const Torque& virtualTorque);

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

 /*!
  * Calculate the joint torques from the desired contact forces
  * with Jacobi transpose.
  */
  bool computeJointTorques();

  bool resetOptimization();

  /*!
   * Update the data that is recorded by the logger.
   * @return true if successful, false if it is not logging.
   */
  bool updateLoggerData();
};

} /* namespace loco */
