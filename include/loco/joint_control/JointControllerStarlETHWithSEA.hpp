/*
 * Copyright (c) 2014, Christian Gehring
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Christian Gehring
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef TESTBED_JOINTCONTROLLERSTARLETH_HPP_
#define TESTBED_JOINTCONTROLLERSTARLETH_HPP_

#include "loco/joint_control/JointControllerBase.hpp"
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>
#include <loco/common/TypeDefsStarlETH.hpp>
#include "GaussianKernelJumpPropagator.hpp"
#include "RobotModel.hpp"
#include <string>
#include <math.h>

#define MAX_POWER 180 //[W]
#define ABSOLUTE_MAX_VELOCITY 10 //[rad/s]
#define ABSOLUTE_MAX_TORQUE 20 //[Nm]

#define PGAIN_HAA 1.25
#define PGAIN_HFE 1.25
#define PGAIN_KFE 1.25

#define TOTAL_NUMBER_OF_JOINTS robotModel_->q().getQj().size() //Get total number of joints from number of minimal coordinates of joints

namespace loco {

typedef kindr::phys_quant::eigen_impl::VectorTypeless<double, 12> JointVector;
typedef kindr::phys_quant::eigen_impl::Torque<double, 12> JointTorques;
typedef kindr::phys_quant::eigen_impl::Position<double, 12> JointPositions;
typedef kindr::phys_quant::eigen_impl::Velocity<double, 12> JointVelocities;
typedef Eigen::Matrix<double, 19, 1> GeneralizedCoordinates;
typedef Eigen::Matrix<double, 18, 1> GeneralizedVelocities;
typedef Eigen::Matrix<double, 18, 1> GeneralizedAccelerations;
typedef Eigen::Matrix<double, 12, 1> JointGains;
typedef Eigen::Matrix<double, 12, 1> SpringStiffnesses;
typedef Eigen::Matrix<double, 12, 1> SpringDampings;

class JointControllerStarlETHWithSEA : JointControllerBase {
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

  const JointTorques& getDesiredJointTorques() const;
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
  virtual void setDesiredJointPositionsInVelocityControl(
      const robotModel::VectorAct& positions);

  void setJointPositionLimitsFromDefaultConfiguration(
      const Eigen::Vector3d& jointMinPositionsForLegInDefaultConfiguration,
      const Eigen::Vector3d& jointMaxPositionsForLegInDefaultConfiguration,
      const bool isLegInDefaultConfiguration[]);

  void setJointTorqueLimitsFromDefaultConfiguration(
      const Eigen::Vector3d& jointMinTorquesForLegInDefaultConfiguration,
      const Eigen::Vector3d& jointMaxTorquesForLegInDefaultConfiguration,
      const bool isLegInDefaultConfiguration[]);

  void setJointVelocityLimitsFromDefaultConfiguration(
      const Eigen::Vector3d& jointMinVelocitiesForLegInDefaultConfiguration,
      const Eigen::Vector3d& jointMaxVelocitiesForLegInDefaultConfiguration,
      const bool isLegInDefaultConfiguration[]);

  const JointPositions& getMaxJointPositions() const;
  const JointPositions& getMinJointPositions() const;

  const JointVelocities& getMaxJointVelocities() const;
  const JointVelocities& getMinJointVelocities() const;

  const JointTorques& getMaxJointTorques() const;
  const JointTorques& getMinJointTorques() const;

  double currentTime_;

  friend std::ostream& operator <<(
      std::ostream& out, const JointControllerStarlETHWithSEA& controller);

 protected:
//  std::ofstream haa_out_;
  std::ofstream hfe_out_;
  std::ofstream kfe_out_;
//  std::ofstream haa_desout_;
  std::ofstream hfe_desout_;
  std::ofstream kfe_desout_;

  robotModel::RobotModel* robotModel_;
  bool isClampingTorques_;
  bool isClampingPositions_;
  bool isClampingVelocities_;

  JointGains pGains_;
  JointGains motorPositionGains_;
  JointGains jointPositionGains_;
  JointGains jointVelocityGains_;
  SpringStiffnesses springStiffnesses_;
  SpringDampings springDampings_;

  JointTorques jointTorquesToSet_;

  JointTorques desJointTorques_;

  JointPositions measuredJointPositions_;
  JointVelocities measuredJointVelocities_;
  JointPositions desJointPositions_;
  JointVelocities desJointVelocities_;

  JointVelocities desMotorVelocities_;
  JointPositions measuredMotorPositions_;
  JointVelocities measuredMotorVelocities_;

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

  double lowPass(int index, double cutOff, double dt);
  double trackJointTorque(int index, double dt);
  double trackMotorVelocity(int index, double dt);
  double calculateTorqueFromSprings(int index, double dt);
  double trackJointPosition(int index, double dt);

 private:
  GaussianKernelJumpPropagator trajectoryFollower_;
};

} /* namespace loco */

#endif /* LOCO_JOINTCONTROLLERSTARLETH_HPP_ */
