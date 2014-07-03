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

namespace loco {

class JointControllerStarlETHWithSEA : public JointControllerBase {

  enum JointTypes {
    HAA = 0,
    HFE,
    KFE
  };

 public:

  typedef kindr::phys_quant::eigen_impl::VectorTypeless<double, 12> JointVector;
 public:
  JointControllerStarlETHWithSEA(robotModel::RobotModel* robotModel);
  virtual ~JointControllerStarlETHWithSEA();

  virtual bool initialize(double dt);
  virtual bool advance(double dt);

  void setJointControlGainsHAA(double kp, double kd);
  void setJointControlGainsHFE(double kp, double kd);
  void setJointControlGainsKFE(double kp, double kd);
  void setMaxTorqueHAA(double maxTorque);
  void setMaxTorqueHFE(double maxTorque);
  void setMaxTorqueKFE(double maxTorque);

  virtual bool loadParameters(const TiXmlHandle& handle);

  virtual void setDesiredJointPositionsInVelocityControl(
      const robotModel::VectorAct& positions);
  void setJointPositionLimitsFromDefaultConfiguration(
      const Eigen::Vector3d& jointMinPositionsForLegInDefaultConfiguration,
      const Eigen::Vector3d& jointMaxPositionsForLegInDefaultConfiguration,
      const bool isLegInDefaultConfiguration[]);

  const JointTorques& getJointTorques() const;
  bool trackMotorTorques(double dt);
  bool trackMotorVelocities(double dt);
  bool transferFunctionMotorVelocitiesToTorques(double dt);

  const JointVector& getJointMaxPositions() const;
  const JointVector& getJointMinPositions() const;
 protected:
  robotModel::RobotModel* robotModel_;

  robotModel::VectorActM desJointModes_;
  JointTorques desJointTorques_;
  robotModel::VectorAct desiredMotorVelocities_;

  JointTorques measuredJointTorques_;
  robotModel::VectorQj measuredMotorVelocities_;
  robotModel::VectorQj measuredMotorPositions_;
  robotModel::VectorQj measuredJointVelocities_;
  robotModel::VectorQj measuredJointPositions_;

  JointTorques previousMeasuredMotorTorque_;

  std::vector<double> springStiffnesses_;
  std::vector<double> springDampings_;

  robotModel::VectorQj& positiveVelocityLimits_;
  robotModel::VectorQj& negativeVelocityLimits_;

  JointVector jointPositionControlProportionalGains_;
  JointVector jointPositionControlDerivativeGains_;
  JointVector jointVelocityControlProportionalGains_;
  JointVector jointVelocityControlDerivativeGains_;
  JointTorques jointMaxTorques_;
  JointVector jointMaxPositions_;
  JointVector jointMinPositions_;
  bool isLegInDefaultConfiguration_[4];
};

} /* namespace loco */

#endif /* LOCO_JOINTCONTROLLERSTARLETHWITHSEA_HPP_ */
