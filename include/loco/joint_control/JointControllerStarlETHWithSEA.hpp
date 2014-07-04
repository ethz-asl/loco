/*
 * JointControllerStarlETHWithSEA.hpp
 *
 *  Created on: Apr 2, 2014
 *      Author: gech
 */

#ifndef LOCO_JOINTCONTROLLERSTARLETHWITHSEA_HPP_
#define LOCO_JOINTCONTROLLERSTARLETHWITHSEA_HPP_

#include "loco/joint_control/JointControllerBase.hpp"
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>
#include <loco/common/TypeDefsStarlETH.hpp>
#include "RobotModel.hpp"

#define MAX_POWER 180 //[W]
#define ABSOLUTE_MAX_VELOCITY 10 //[rad/s]
#define ABSOLUTE_MAX_TORQUE 20 //[Nm]
#define DELTA_LIMIT 0.5 //[rad]

#define PGAIN_HAA 38.0
#define PGAIN_HFE 38.0
#define PGAIN_KFE 20.0

#define TOTAL_NUMBER_OF_JOINTS robotModel_->q().getQj().size() //Get total number of joints from number of minimal coordinates of joints

namespace loco {

typedef kindr::phys_quant::eigen_impl::VectorTypeless<double, 12> JointVector;
typedef kindr::phys_quant::eigen_impl::Torque<double, 12> JointTorques;
typedef kindr::phys_quant::eigen_impl::Position<double, 12> JointPositions;
typedef kindr::phys_quant::eigen_impl::Velocity<double, 12> JointVelocities;
typedef Eigen::Matrix<double, 19, 1> GeneralizedCoordinates;
typedef Eigen::Matrix<double, 18, 1> GeneralizedVelocities;
typedef Eigen::Matrix<double, 18, 1> GeneralizedAccelerations;
typedef Eigen::Matrix<double, 12, 1> JointPGains;
typedef Eigen::Matrix<double, 12, 1> SpringStiffnesses;
typedef Eigen::Matrix<double, 12, 1> SpringDampings;

class JointControllerStarlETHWithSEA : public JointControllerBase {

  enum JointTypes {
    HAA = 0,
    HFE,
    KFE,
    NUMBER_OF_JOINT_TYPES,
  };

 public:
  JointControllerStarlETHWithSEA(robotModel::RobotModel* robotModel);
  virtual ~JointControllerStarlETHWithSEA();

  virtual bool initialize(double dt);
  virtual bool advance(double dt);

  const JointTorques& getJointTorques() const;
  const JointPositions& getJointPositions() const;
  const JointVelocities& getJointVelocities() const;

  void setJointControlGainsHAA(double kp, double kd);
  void setJointControlGainsHFE(double kp, double kd);
  void setJointControlGainsKFE(double kp, double kd);
  void setMaxTorqueHAA(double maxTorque);
  void setMaxTorqueHFE(double maxTorque);
  void setMaxTorqueKFE(double maxTorque);
  void setMinTorqueHAA(double minTorque);
  void setMinTorqueHFE(double minTorque);
  void setMinTorqueKFE(double minTorque);

  virtual void setIsClampingTorques(bool sClamping);
  virtual void setIsClampingPositions(bool isClamping);
  virtual void setIsClampingVelocities(bool isClamping);

  virtual bool loadParameters(const TiXmlHandle& handle);
  virtual void setDesiredJointPositionsInVelocityControl(const robotModel::VectorAct& positions);

  void setJointPositionLimitsFromDefaultConfiguration(
      const Eigen::Vector3d& jointMinPositionsForLegInDefaultConfiguration,
      const Eigen::Vector3d& jointMaxPositionsForLegInDefaultConfiguration,
      const bool isLegInDefaultConfiguration[]);

  void setJointTorqueLimitsFromDefaultConfiguration(const Eigen::Vector3d& jointMinTorquesForLegInDefaultConfiguration,
                                                      const Eigen::Vector3d& jointMaxTorquesForLegInDefaultConfiguration,
                                                      const bool isLegInDefaultConfiguration[]);

    void setJointVelocityLimitsFromDefaultConfiguration(const Eigen::Vector3d& jointMinVelocitiesForLegInDefaultConfiguration,
                                                        const Eigen::Vector3d& jointMaxVelocitiesForLegInDefaultConfiguration,
                                                        const bool isLegInDefaultConfiguration[]);

    const JointPositions& getMaxJointPositions() const;
    const JointPositions& getMinJointPositions() const;

    const JointVelocities& getMaxJointVelocities() const;
    const JointVelocities& getMinJointVelocities() const;

    const JointTorques& getMaxJointTorques() const;
    const JointTorques& getMinJointTorques() const;

    friend std::ostream& operator << (std::ostream& out, const JointControllerStarlETHWithSEA& controller);

    bool trackMotorTorques(double dt);

 protected:
  bool trackMotorVelocities(double dt);
  bool transferFunctionMotorVelocitiesToTorques(double dt);

  robotModel::RobotModel* robotModel_;

  bool isClampingTorques_;
  bool isClampingPositions_;
  bool isClampingVelocities_;

  robotModel::VectorActM desJointModes_;
  JointTorques desJointTorques_;
  JointVelocities desiredMotorVelocities_;

  JointTorques measuredJointTorques_;
  JointVelocities measuredMotorVelocities_;
  JointPositions measuredMotorPositions_;
  JointVelocities measuredJointVelocities_;
  JointPositions measuredJointPositions_;

  JointTorques previousMeasuredJointTorques_;

  JointPGains pGains_;
  SpringStiffnesses springStiffnesses_;
  SpringDampings springDampings_;

  JointTorques jointTorques_;
  JointPositions jointPositions_;
  JointVelocities jointVelocities_;

  //! desired positions used in the low-level velocity control mode
  robotModel::VectorAct desPositionsInVelocityControl_;
  robotModel::VectorActM desJointModesPrevious_;

  JointVector jointPositionControlProportionalGains_;
  JointVector jointPositionControlDerivativeGains_;
  JointVector jointVelocityControlProportionalGains_;
  JointVector jointVelocityControlDerivativeGains_;
  JointTorques jointMaxTorques_;
  JointTorques jointMinTorques_;
  JointPositions jointMaxPositions_;
  JointPositions jointMinPositions_;
  JointVelocities jointMaxVelocities_;
  JointVelocities jointMinVelocities_;

  bool isLegInDefaultConfiguration_[4];
};

} /* namespace loco */

#endif /* LOCO_JOINTCONTROLLERSTARLETHWITHSEA_HPP_ */
