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
#include <Eigen/SparseCore>
#include "RobotModel.hpp"

namespace robotController {

class ContactForceDistribution
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Leg enum.
   * TODO move this to robot commons?
   * TODO make enum class
   */
  enum LegNumber {
    LEFT_FRONT = 0,
    RIGHT_FRONT,
    LEFT_HIND,
    RIGHT_HIND
  };

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
   * @return true if successful
   */
  bool loadParameters();

  /*!
   * Computes the contact force distribution of the virtual force and torque.
   * @param virtualForce the desired virtual force (in base frame)
   * @param virtualTorque the desired virtual torque (in base frame)
   * @return true if successful
   */
  bool computeForceDistribution(const Eigen::Vector3d& virtualForce,
                                const Eigen::Vector3d& virtualTorque);

  /*!
   * Manually change how much a leg should be loaded.
   * This needs to be set before calling computeForceDistribution() and
   * is reset after each call of computeForceDistribution().
   * @param legNumber defines the leg
   * @param loadFactor sets the factor how much the leg should be loaded
   *        value in the interval [0, 1] where 0: unloaded and 1: completely loaded
   * @return true if successful, false if leg is not in contact with the ground (unless loadFactor=0.0)
   */
  bool changeLegLoad(const LegNumber& legNumber, const double& loadFactor);

  /*!
   * Gets the distributed force for a leg at the contact point
   * @param [in] legNumber defines the leg
   * @param [out] force is the distributed force on the leg (in base frame)
   * @return true if leg is active (in stance) and force can be applied, false otherwise
   */
  bool getForceForLeg(const LegNumber& legNumber, robotModel::VectorCF& force);

  /*!
   * TODO
   * Gets the distributed net forces and torques that act on the main body, i.e.
   * these forces and torques are computed from the distributed forces and should be equal to
   * the desired net forces and torques.
   * @param netForce
   * @param netTorque
   */
  // void getDistributedNetForces(Vector3d& netForce, Vector3d& netTorque);

 private:
  const int nLegs_ = 4; // TODO move to robotCommons as constexpr
  const int nTranslationalDofPerFoot_ = 3; // TODO define as constexpr
  const int nElementsInStackedVirtualForceTorqueVector_ = 2 * nTranslationalDofPerFoot_; // TODO define as constexpr
  const int nConstraintsPerFoot_ = 5; // TODO define as constexpr

  //! Reference to robot model
  robotModel::RobotModel* robotModel_;

  //! True when parameters are successfully loaded.
  bool isParametersLoaded_;

  //! Leg load factors
  Eigen::VectorXd legLoadFactors_;
  //! Number of legs in stance phase
  int nLegsInStance_;
  //! Number of variables to optimize (size of x, n = nTranslationalDofPerFoot_ * nLegsInStance_)
  int n_;

  //! Diagonal elements of the weighting matrix for the desired virtual forces and torques (for S).
  Eigen::VectorXd virtualForceWeights_;
  //! Diagonal element of the weighting matrix for the ground reaction forces (regularizer, for W).
  double groundForceWeight_;
  //! Minimal normal ground force (F_min^n, in N).
  double minimalNormalGroundForce_;
  //! Assumed friction coefficient (mu).
  double frictionCoefficient_;

  //! The matrix A in the optimization formulation (nElementsInStackedVirtualForceTorqueVector_ x n).
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_;
  //! The vector b in the optimization formulation (stacked vector of desired net virtual forces and torques).
  Eigen::VectorXd b_;
  //! Force constraint matrix
  Eigen::SparseMatrix<double, Eigen::RowMajor> D_;
  //! Upper and lower limits vectors
  Eigen::VectorXd d_, f_;
  //! Stacked contact forces
  Eigen::VectorXd x_;
  //! Weighting matrix for the ground reaction forces (regularizer).
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> W_;
  //! Weighting matrix for the desired virtual forces and torques.
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> S_;

  //  Eigen::SparseMatrix<double, Eigen::RowMajor> C;
  //  Eigen::VectorXd c;

  struct LegStatus
  {
    bool isInStance_;
    double loadFactor_;
    int indexInStanceLegList_;
    int startIndexInVectorX_;
  };

  std::map<LegNumber, LegStatus> legStatuses_;

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
  /*!
   * Solve optimization
   * @return true if successful
   */
  bool solveOptimization();

  bool resetFootLoadFactors();


};

} /* namespace robotController */
#endif /* CONTACTFORCEDISTRIBUTION_HPP_ */
