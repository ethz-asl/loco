/*
 * ContactForceDistributionBase.hpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <Eigen/Core>
#include "RobotModel.hpp"
#include "TerrainBase.hpp"
#include "loco/temp_helpers/Legs.hpp"

namespace loco {

//! This class distributes a virtual force and torque on the base as forces to the leg contact points.
class ContactForceDistributionBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Constructor.
   * @param robotModel the reference to the robot.
   */
  ContactForceDistributionBase(robotModel::RobotModel* robotModel);

  /*!
   * Destructor.
   */
  virtual ~ContactForceDistributionBase();

  /*!
   * Loads the parameters. Has to be done before using this class.
   * @return true if successful.
   */
  virtual bool loadParameters();

  /*!
   * Set the terrain to get the surface normal at a certain foot.
   * This information is used in the contact force distribution.
   * @param[in] terrain the reference to the terrain object.
   * @return true if successful.
   */
  virtual bool setTerrain(const robotTerrain::TerrainBase* terrain);


  /*!
   * Adds class data to the logger (optional).
   * @return true if successful.
   */
  virtual bool addToLogger();

  /*!
   * Computes the contact force distribution of the virtual force and torque.
   * @param virtualForce the desired virtual force on the base (in base frame).
   * @param virtualTorque the desired virtual torque on the base (in base frame).
   * @return true if successful.
   */
  virtual bool computeForceDistribution(const Eigen::Vector3d& virtualForce,
                                        const Eigen::Vector3d& virtualTorque) = 0;

  /*!
   * Gets the distributed force for a leg at the contact point.
   * @param[in] leg defines the leg.
   * @param[out] force is the distributed force on the leg (in base frame).
   * @return true if leg is active (in stance) and force can be applied, false otherwise.
   */
  virtual bool getForceForLeg(const Legs& leg, robotModel::VectorCF& force) = 0;

  /*!
   * Gets the distributed net forces and torques that act on the base, i.e.
   * this force and torque are computed from the distributed contact forces and
   * should ideally be equal to the desired net forces and torques.
   * @param[out] netForce.
   * @param[out] netTorque.
   * @return true if net force and torque can be calculated, false otherwise.
   */
   virtual bool getNetForceAndTorqueOnBase(Eigen::Vector3d& netForce, Eigen::Vector3d& netTorque) = 0;

 protected:
  constexpr static int nLegs_ = 4; // TODO move to robotModel
  constexpr static int nTranslationalDofPerFoot_ = 3; // TODO move to robotModel
  constexpr static int nElementsVirtualForceTorqueVector_ = 6;

  //! Reference to robot model
  robotModel::RobotModel* robotModel_;

  //! Reference to the terrain
  const robotTerrain::TerrainBase* terrain_;

  //! True if parameters are successfully loaded.
  bool isParametersLoaded_;

  //! True if data is logged.
  bool isLogging_;

  //! True if terrain is set.
  bool isTerrainSet_;

  //! True if a force distribution was computed successfully.
  bool isForceDistributionComputed_;

  /*!
   * Check if parameters are loaded.
   * @return true if parameters are loaded.
   */
  virtual bool checkIfParametersLoaded() const;

  /*!
   * Check if terrain is set.
   * @return true is set.
   */
  virtual bool checkIfTerrainSet() const;

  /*!
   * Check if force distribution is computed.
   * @return true if force distribution is computed.
   */
  virtual bool checkIfForceDistributionComputed() const;

  /*!
   * Update the data that is recorded by the logger.
   * @return true if successful, false if it is not logging.
   */
  virtual bool updateLoggerData() = 0;
};

} /* namespace loco */
