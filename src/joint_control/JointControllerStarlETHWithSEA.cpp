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
#include "loco/joint_control/JointControllerStarlETHWithSEA.hpp"
#include <limits>
#include <iostream>

namespace loco {

JointControllerStarlETHWithSEA::JointControllerStarlETHWithSEA(
    robotModel::RobotModel* robotModel)
    : robotModel_(robotModel),
      isClampingTorques_(true),
      isClampingPositions_(false),
      isClampingVelocities_(false) {

  /* set gains */
  setJointControlGainsHAA(70.0, 3.1);
  setJointControlGainsHFE(70.0, 3.2);
  setJointControlGainsKFE(26.0, 0.2);

  /* X-Configuration */
  isLegInDefaultConfiguration_[0] = true;
  isLegInDefaultConfiguration_[1] = true;
  isLegInDefaultConfiguration_[2] = false;
  isLegInDefaultConfiguration_[3] = false;

  /* set position limits */
  const double deltaLimit = 0.05;  // [rad]r
  Eigen::Vector3d jointMinPositionsForLegInDefaultConfiguration;
  Eigen::Vector3d jointMaxPositionsForLegInDefaultConfiguration;
  jointMinPositionsForLegInDefaultConfiguration(0) = -0.4 + deltaLimit;  // -0.2+deltaLimit;
  jointMaxPositionsForLegInDefaultConfiguration(0) = 0.5 - deltaLimit;  //0.5
  jointMinPositionsForLegInDefaultConfiguration(1) = -0.6 + deltaLimit;
  jointMaxPositionsForLegInDefaultConfiguration(1) = 1.29 - deltaLimit;  //1.29  1.29-2*deltaLimit;
  jointMinPositionsForLegInDefaultConfiguration(2) = -2.46 + deltaLimit;  //-2.46
  jointMaxPositionsForLegInDefaultConfiguration(2) = -0.1 - deltaLimit;  // -0.1
  setJointPositionLimitsFromDefaultConfiguration(
      jointMinPositionsForLegInDefaultConfiguration,
      jointMaxPositionsForLegInDefaultConfiguration,
      isLegInDefaultConfiguration_);

  /* set torque limits */
  const double absMaxTorque = 20.0;  // [Nm]
  Eigen::Vector3d jointMinTorquesForLegInDefaultConfiguration;
  Eigen::Vector3d jointMaxTorquesForLegInDefaultConfiguration;
  jointMinTorquesForLegInDefaultConfiguration(0) = -absMaxTorque;
  jointMaxTorquesForLegInDefaultConfiguration(0) = absMaxTorque;
  jointMinTorquesForLegInDefaultConfiguration(1) = -absMaxTorque;
  jointMaxTorquesForLegInDefaultConfiguration(1) = absMaxTorque;
  jointMinTorquesForLegInDefaultConfiguration(2) = -0.0;
  jointMaxTorquesForLegInDefaultConfiguration(2) = absMaxTorque;
  setJointTorqueLimitsFromDefaultConfiguration(
      jointMinTorquesForLegInDefaultConfiguration,
      jointMaxTorquesForLegInDefaultConfiguration,
      isLegInDefaultConfiguration_);

  /* set velocity limits */
  const double absMaxVelocity = 10.0;  // [m/s]
  Eigen::Vector3d jointMinVelocitiesForLegInDefaultConfiguration;
  Eigen::Vector3d jointMaxVelocitiesForLegInDefaultConfiguration;
  jointMinVelocitiesForLegInDefaultConfiguration(0) = -absMaxVelocity;
  jointMaxVelocitiesForLegInDefaultConfiguration(0) = absMaxVelocity;
  jointMinVelocitiesForLegInDefaultConfiguration(1) = -absMaxVelocity;
  jointMaxVelocitiesForLegInDefaultConfiguration(1) = absMaxVelocity;
  jointMinVelocitiesForLegInDefaultConfiguration(2) = -absMaxVelocity;
  jointMaxVelocitiesForLegInDefaultConfiguration(2) = absMaxVelocity;
  setJointVelocityLimitsFromDefaultConfiguration(
      jointMinVelocitiesForLegInDefaultConfiguration,
      jointMaxVelocitiesForLegInDefaultConfiguration,
      isLegInDefaultConfiguration_);
}

void JointControllerStarlETHWithSEA::setJointPositionLimitsFromDefaultConfiguration(
    const Eigen::Vector3d& jointMinPositionsForLegInDefaultConfiguration,
    const Eigen::Vector3d& jointMaxPositionsForLegInDefaultConfiguration,
    const bool isLegInDefaultConfiguration[]) {

  //  LF_LEG limits
  jointMinPositions_(0) = jointMinPositionsForLegInDefaultConfiguration(0);
  jointMaxPositions_(0) = jointMaxPositionsForLegInDefaultConfiguration(0);

  //  RF_LEG limits
  jointMinPositions_(3) = -jointMaxPositionsForLegInDefaultConfiguration(0);
  jointMaxPositions_(3) = -jointMinPositionsForLegInDefaultConfiguration(0);

  //  LH_LEG limits
  jointMinPositions_(6) = jointMinPositionsForLegInDefaultConfiguration(0);
  jointMaxPositions_(6) = jointMaxPositionsForLegInDefaultConfiguration(0);

  //  RH_LEG limits
  jointMinPositions_(9) = -jointMaxPositionsForLegInDefaultConfiguration(0);
  jointMaxPositions_(9) = -jointMinPositionsForLegInDefaultConfiguration(0);

  for (int iLeg = 0; iLeg < 4; iLeg++) {
    jointMinPositions_(1 + iLeg * 3) =
        isLegInDefaultConfiguration[iLeg] ?
            jointMinPositionsForLegInDefaultConfiguration(1) :
            -jointMaxPositionsForLegInDefaultConfiguration(1);
    jointMaxPositions_(1 + iLeg * 3) =
        isLegInDefaultConfiguration[iLeg] ?
            jointMaxPositionsForLegInDefaultConfiguration(1) :
            -jointMinPositionsForLegInDefaultConfiguration(1);
    jointMinPositions_(2 + iLeg * 3) =
        isLegInDefaultConfiguration[iLeg] ?
            jointMinPositionsForLegInDefaultConfiguration(2) :
            -jointMaxPositionsForLegInDefaultConfiguration(2);
    jointMaxPositions_(2 + iLeg * 3) =
        isLegInDefaultConfiguration[iLeg] ?
            jointMaxPositionsForLegInDefaultConfiguration(2) :
            -jointMinPositionsForLegInDefaultConfiguration(2);
  }
}

void JointControllerStarlETHWithSEA::setJointTorqueLimitsFromDefaultConfiguration(
    const Eigen::Vector3d& jointMinTorquesForLegInDefaultConfiguration,
    const Eigen::Vector3d& jointMaxTorquesForLegInDefaultConfiguration,
    const bool isLegInDefaultConfiguration[]) {

  //  LF_LEG limits
  jointMinTorques_(0) = jointMinTorquesForLegInDefaultConfiguration(0);
  jointMaxTorques_(0) = jointMaxTorquesForLegInDefaultConfiguration(0);

  //  RF_LEG limits
  jointMinTorques_(3) = -jointMaxTorquesForLegInDefaultConfiguration(0);
  jointMaxTorques_(3) = -jointMinTorquesForLegInDefaultConfiguration(0);

  //  LH_LEG limits
  jointMinTorques_(6) = jointMinTorquesForLegInDefaultConfiguration(0);
  jointMaxTorques_(6) = jointMaxTorquesForLegInDefaultConfiguration(0);

  //  RH_LEG limits
  jointMinTorques_(9) = -jointMaxTorquesForLegInDefaultConfiguration(0);
  jointMaxTorques_(9) = -jointMinTorquesForLegInDefaultConfiguration(0);

  for (int iLeg = 0; iLeg < 4; iLeg++) {
    jointMinTorques_(1 + iLeg * 3) =
        isLegInDefaultConfiguration[iLeg] ?
            jointMinTorquesForLegInDefaultConfiguration(1) :
            -jointMaxTorquesForLegInDefaultConfiguration(1);
    jointMaxTorques_(1 + iLeg * 3) =
        isLegInDefaultConfiguration[iLeg] ?
            jointMaxTorquesForLegInDefaultConfiguration(1) :
            -jointMinTorquesForLegInDefaultConfiguration(1);
    jointMinTorques_(2 + iLeg * 3) =
        isLegInDefaultConfiguration[iLeg] ?
            jointMinTorquesForLegInDefaultConfiguration(2) :
            -jointMaxTorquesForLegInDefaultConfiguration(2);
    jointMaxTorques_(2 + iLeg * 3) =
        isLegInDefaultConfiguration[iLeg] ?
            jointMaxTorquesForLegInDefaultConfiguration(2) :
            -jointMinTorquesForLegInDefaultConfiguration(2);
  }
}

void JointControllerStarlETHWithSEA::setJointVelocityLimitsFromDefaultConfiguration(
    const Eigen::Vector3d& jointMinVelocitiesForLegInDefaultConfiguration,
    const Eigen::Vector3d& jointMaxVelocitiesForLegInDefaultConfiguration,
    const bool isLegInDefaultConfiguration[]) {
  //  LF_LEG limits
  jointMinVelocities_(0) = jointMinVelocitiesForLegInDefaultConfiguration(0);
  jointMaxVelocities_(0) = jointMaxVelocitiesForLegInDefaultConfiguration(0);

  //  RF_LEG limits
  jointMinVelocities_(3) = -jointMaxVelocitiesForLegInDefaultConfiguration(0);
  jointMaxVelocities_(3) = -jointMinVelocitiesForLegInDefaultConfiguration(0);

  //  LH_LEG limits
  jointMinVelocities_(6) = jointMinVelocitiesForLegInDefaultConfiguration(0);
  jointMaxVelocities_(6) = jointMaxVelocitiesForLegInDefaultConfiguration(0);

  //  RH_LEG limits
  jointMinVelocities_(9) = -jointMaxVelocitiesForLegInDefaultConfiguration(0);
  jointMaxVelocities_(9) = -jointMinVelocitiesForLegInDefaultConfiguration(0);

  for (int iLeg = 0; iLeg < 4; iLeg++) {
    jointMinVelocities_(1 + iLeg * 3) =
        isLegInDefaultConfiguration[iLeg] ?
            jointMinVelocitiesForLegInDefaultConfiguration(1) :
            -jointMaxVelocitiesForLegInDefaultConfiguration(1);
    jointMaxVelocities_(1 + iLeg * 3) =
        isLegInDefaultConfiguration[iLeg] ?
            jointMaxVelocitiesForLegInDefaultConfiguration(1) :
            -jointMinVelocitiesForLegInDefaultConfiguration(1);
    jointMinVelocities_(2 + iLeg * 3) =
        isLegInDefaultConfiguration[iLeg] ?
            jointMinVelocitiesForLegInDefaultConfiguration(2) :
            -jointMaxVelocitiesForLegInDefaultConfiguration(2);
    jointMaxVelocities_(2 + iLeg * 3) =
        isLegInDefaultConfiguration[iLeg] ?
            jointMaxVelocitiesForLegInDefaultConfiguration(2) :
            -jointMinVelocitiesForLegInDefaultConfiguration(2);
  }
}

const JointPositions& JointControllerStarlETHWithSEA::getMaxJointPositions() const {
  return jointMaxPositions_;
}
const JointPositions& JointControllerStarlETHWithSEA::getMinJointPositions() const {
  return jointMinPositions_;
}
const JointVelocities& JointControllerStarlETHWithSEA::getMaxJointVelocities() const {
  return jointMaxVelocities_;
}
const JointVelocities& JointControllerStarlETHWithSEA::getMinJointVelocities() const {
  return jointMinVelocities_;
}
const JointTorques& JointControllerStarlETHWithSEA::getMaxJointTorques() const {
  return jointMaxTorques_;
}
const JointTorques& JointControllerStarlETHWithSEA::getMinJointTorques() const {
  return jointMinTorques_;
}

JointControllerStarlETHWithSEA::~JointControllerStarlETHWithSEA() {
//  haa_out_.close();
  hfe_out_.close();
  kfe_out_.close();
//
//  haa_desout_.close();
//  hfe_desout_.close();
//  kfe_desout_.close();
}

void JointControllerStarlETHWithSEA::setDesiredJointPositionsInVelocityControl(
    const robotModel::VectorAct& positions) {
  desPositionsInVelocityControl_ = positions;
}

void JointControllerStarlETHWithSEA::setIsClampingTorques(bool isClamping) {
  isClampingTorques_ = isClamping;
}

void JointControllerStarlETHWithSEA::setIsClampingPositions(bool isClamping) {
  isClampingPositions_ = isClamping;
}

void JointControllerStarlETHWithSEA::setIsClampingVelocities(bool isClamping) {
  isClampingVelocities_ = isClamping;
}

bool JointControllerStarlETHWithSEA::initialize(double dt) {
  desJointModesPrevious_ = robotModel_->act().getMode();
  desPositionsInVelocityControl_ = robotModel_->q().getQj();  //robotModel_->act().getPos();
  desJointTorques_.setZero();
  jointTorquesToSet_.setZero();
  desMotorVelocities_.setZero();
  measuredMotorVelocities_.setZero();
  desJointModesPrevious_.fill(robotModel::AM_Velocity);

  measuredMotorPositions_ = JointPositions(robotModel_->q().getQj());

//  haa_out_.open("haa_joint_torques_in_torque_mode");
  hfe_out_.open("hfe_joint_torques_in_torque_mode");
  kfe_out_.open("kfe_joint_torques_in_torque_mode");
//
//  haa_desout_.open("haa_joint_destorques_in_torque_mode");
//  hfe_desout_.open("hfe_joint_destorques_in_torque_mode");
//  kfe_desout_.open("kfe_joint_destorques_in_torque_mode");

  for (int i = 0; i < TOTAL_NUMBER_OF_JOINTS; i +=
      (int) JointTypes::NUMBER_OF_JOINT_TYPES) {

    springStiffnesses_[JointTypes::HAA + i] = robotModel_->params()
        .springStiffnesses_.haa;
    springStiffnesses_[JointTypes::HFE + i] = robotModel_->params()
        .springStiffnesses_.hfe;
    springStiffnesses_[JointTypes::KFE + i] = robotModel_->params()
        .springStiffnesses_.kfe;

    springDampings_[JointTypes::HAA + i] = robotModel_->params().springDampings_
        .haa;
    springDampings_[JointTypes::HFE + i] = robotModel_->params().springDampings_
        .hfe;
    springDampings_[JointTypes::KFE + i] = robotModel_->params().springDampings_
        .kfe;

    // Set gains for position control
    motorPositionGains_[JointTypes::HAA + i] = 30.0;
    motorPositionGains_[JointTypes::HFE + i] = 50.0;
    motorPositionGains_[JointTypes::KFE + i] = 70.0;

    jointPositionGains_[JointTypes::HAA + i] = -17;
    jointPositionGains_[JointTypes::HFE + i] = -20;
    jointPositionGains_[JointTypes::KFE + i] = 0.0;

    jointVelocityGains_[JointTypes::HAA + i] = 0.0;
    jointVelocityGains_[JointTypes::HFE + i] = 0.6;
    jointVelocityGains_[JointTypes::KFE + i] = 0.0;

    // Set P-gains for torque control
    pGains_[JointTypes::HAA + i] = PGAIN_HAA;
    pGains_[JointTypes::HFE + i] = PGAIN_HFE;
    pGains_[JointTypes::KFE + i] = PGAIN_KFE;

//    std::cout << "JointController: " << std::endl;
//    std::cout << "MAX: " << jointMaxPositions_ << std::endl;
//    std::cout << "MIN: " << jointMinPositions_ << std::endl;
  }

  return true;
}

/**
 * Low-pass filter for desired velocity.
 */
double JointControllerStarlETHWithSEA::lowPass(int index, double cutOff,
                                               double dt) {
  double a0, b0;

  a0 = dt / (1 / cutOff + dt);
  b0 = (1 / cutOff) / (1 / cutOff + dt);

  return desMotorVelocities_(index) * b0 + measuredMotorVelocities_(index) * a0;
}

/**
 * Calculates torque according to actuation model with springs.
 * Calculation is done by taking the spring deflections in position ands velocity and multiply them with spring stiffnesses and dampings.
 * @param
 */
double JointControllerStarlETHWithSEA::calculateTorqueFromSprings(int index,
                                                                  double dt) {

  return springStiffnesses_(index)
      * (measuredMotorPositions_(index) - measuredJointPositions_(index))
      + springDampings_(index)
          * (measuredMotorVelocities_(index) - measuredJointVelocities_(index));
}

/**
 * Transfer function for motor dynamics (from desired motor velocity to measured motor velocity).
 * Tracks a given reference motor velocity.
 * @param index: index of current joint
 * @param dt: time step
 * @param desMotorVelocity: velocity reference for current joint
 */
double JointControllerStarlETHWithSEA::trackMotorVelocity(int index,
                                                          double dt) {

  /* Without low pass */
//  double measuredMotorVelocity = desMotorVelocities_(i);
  desMotorVelocities_(index) = lowPass(index, 100, dt);

  double maxVelocity;

  // Set max velocity to absolute max velocity (physical limit) if previous torque is zero.
  // Otherwise, calculate max velocity by using max power
  maxVelocity =
      jointTorquesToSet_(index) == 0.0 ?
          ABSOLUTE_MAX_VELOCITY :
          MAX_POWER / std::abs(jointTorquesToSet_(index));

  // Clamp velocity according to power limit.
  if (desMotorVelocities_(index) > maxVelocity) {
    desMotorVelocities_(index) = maxVelocity;
  } else if (desMotorVelocities_(index) < -maxVelocity) {
    desMotorVelocities_(index) = -maxVelocity;
  }

  // Make sure the velocity is still within bounds of physical limits.
  if (desMotorVelocities_(index) > jointMaxVelocities_(index)) {
    desMotorVelocities_(index) = jointMaxVelocities_(index);
  } else if (desMotorVelocities_(index) < jointMinVelocities_(index)) {
    desMotorVelocities_(index) = jointMinVelocities_(index);
  }

  measuredMotorPositions_(index) += desMotorVelocities_(index) * dt;

  return desMotorVelocities_(index);
}

/**
 * Tracks reference torque as defined by desJointTorques.
 * @param index: index of current joint
 * @param dt: time step
 */
double JointControllerStarlETHWithSEA::trackJointTorque(int index, double dt) {
  double torqueToSet;

  desMotorVelocities_(index) = (desJointTorques_(index)
      - jointTorquesToSet_(index)) * pGains_(index);

  measuredMotorVelocities_(index) = trackMotorVelocity(index, dt);

  torqueToSet = calculateTorqueFromSprings(index, dt);

  return torqueToSet;
}

/**
 * Tracks reference position as defined by desJointPosition.
 * @param index: index of current joint
 * @param dt: time step
 */
double JointControllerStarlETHWithSEA::trackJointPosition(int index,
                                                          double dt) {
  double torqueToSet;

  desMotorVelocities_(index) = (desJointPositions_(index)
      - measuredMotorPositions_(index)) * motorPositionGains_(index)
      + (desJointPositions_(index) - measuredJointPositions_(index))
          * jointPositionGains_(index)
      + (-measuredJointVelocities_(index)) * jointVelocityGains_(index);

  measuredMotorVelocities_(index) = trackMotorVelocity(index, dt);

  torqueToSet = calculateTorqueFromSprings(index, dt);

  return torqueToSet;
}

bool JointControllerStarlETHWithSEA::advance(double dt) {

  desJointPositions_ = JointPositions(robotModel_->act().getPos());
  desJointVelocities_ = JointVelocities(robotModel_->act().getVel());
  const robotModel::VectorActM desJointModes = robotModel_->act().getMode();
  const robotModel::VectorAct desJointTorques = robotModel_->act().getTau();
  const robotModel::VectorQj measJointPositions = robotModel_->q().getQj();
  const robotModel::VectorQj measJointVelocities = robotModel_->q().getdQj();

  robotModel::VectorQj measMotorPositions;
  robotModel::VectorQj measMotorVelocities;
  robotModel::VectorAct jointTorquesToSet;

  static double currentTime = 0;

//  if (currentTime < 1) {
    for (int i = 0; i < desJointModes.size(); i++) {
      measuredJointVelocities_(i) = measJointVelocities(i);
      measuredJointPositions_(i) = measJointPositions(i);

      if (desJointModes(i) == robotModel::AM_Position) {
        if (isClampingPositions_) {
          if (desJointPositions_(i) > jointMaxPositions_(i)) {
            desJointPositions_(i) = jointMaxPositions_(i);
          } else if (desJointPositions_(i) < jointMinPositions_(i)) {
            desJointPositions_(i) = jointMinPositions_(i);
          }
        }

        jointTorquesToSet_(i) = trackJointPosition(i, dt);

//        hfe_desout_ << currentTime << " " << jointTorquesToSet_(1) << std::endl;
//        kfe_desout_ << currentTime << " " << jointTorquesToSet_(2) << std::endl;

      } else if (desJointModes(i) == robotModel::AM_Velocity) {
        if (isClampingVelocities_) {
          if (desJointVelocities_(i) > jointMaxVelocities_(i)) {
            desJointVelocities_(i) = jointMaxVelocities_(i);
          } else if (desJointVelocities_(i) < jointMinVelocities_(i)) {
            desJointVelocities_(i) = jointMinVelocities_(i);
          }
        }

        desMotorVelocities_(i) = desJointVelocities_(i);
        measuredMotorVelocities_(i) = trackMotorVelocity(i, dt);
        jointTorquesToSet_(i) = calculateTorqueFromSprings(i, dt);

      } else if (desJointModes(i) == robotModel::AM_Torque) {
        // Remember the reference signal so we can get it from outside this class if needed.
        // Also needed to track joint torque.

        desJointTorques_(i) = desJointTorques(i);

        if (isClampingTorques_) {
          if (desJointTorques_(i) > jointMaxTorques_(i)) {
            desJointTorques_(i) = jointMaxTorques_(i);
          } else if (desJointTorques_(i) < jointMinTorques_(i)) {
            desJointTorques_(i) = jointMinTorques_(i);
          }
        }

//      haa_desout_ << desJointTorques_(0) << std::endl;
//        hfe_desout_ << currentTime << " " << desJointTorques_(1) << std::endl;
//        kfe_desout_ << currentTime << " " << desJointTorques_(2) << std::endl;

        jointTorquesToSet_(i) = trackJointTorque(i, dt);

      } else {
        throw "Joint control mode is not supported!";
      }

      jointTorquesToSet(i) = jointTorquesToSet_(i);
      measMotorPositions(i) = measuredMotorPositions_(i);
      measMotorVelocities(i) = measuredMotorVelocities_(i);
    }
//  }

//  hfe_desout_ << currentTime << " " << desJointTorques_(1) << std::endl;
//  kfe_desout_ << currentTime << " " << desJointTorques_(2) << std::endl;

//  if (currentTime >= 1) {
//    jointTorquesToSet_(1) = 20;
//    jointTorquesToSet_(2) = 20;
//
//    jointTorquesToSet_(4) = 20;
//    jointTorquesToSet_(5) = 20;
//
//    jointTorquesToSet_(7) = -20;
//    jointTorquesToSet_(8) = -20;
//
//    jointTorquesToSet_(10) = -20;
//    jointTorquesToSet_(11) = -20;
//  }

  for (int i = 0; i < jointTorquesToSet.size(); i++)
    jointTorquesToSet(i) = jointTorquesToSet_(i);

//  std::cout << currentTime << std::endl;
  robotModel_->sensors().setMotorPos(measMotorPositions);
  robotModel_->sensors().setMotorVel(measMotorVelocities);
  robotModel_->sensors().setJointTorques(jointTorquesToSet);

  /* Output the torques that are set */
//  haa_out_ << currentTime << " " << jointTorquesToSet_(0) << std::endl;
  hfe_out_ << currentTime << " " << jointTorquesToSet_(1) << std::endl;
  kfe_out_ << currentTime << " " << jointTorquesToSet_(2) << std::endl;
  /* Output the measured joint positions set */
//  haa_out_ << currentTime << " " << measuredJointPositions_(0)  << std::endl;
//  hfe_out_ << currentTime << " " << measuredJointPositions_(1) << std::endl;
//  kfe_out_ << currentTime << " " << measuredJointPositions_(2) << std::endl;
  /* Output the measured motor velocities set */
  //  haa_out_ << measuredMotorVelocities_(0) << std::endl;
//  hfe_out_ << measuredMotorVelocities_(1) << std::endl;
//  kfe_out_ << measuredMotorVelocities_(2) << std::endl;
  desJointModesPrevious_ = desJointModes;

  currentTime += dt;

  return true;
}

const JointTorques& JointControllerStarlETHWithSEA::getJointTorques() const {
  return jointTorquesToSet_;
}

const JointTorques& JointControllerStarlETHWithSEA::getDesiredJointTorques() const {
  return desJointTorques_;
}

const JointPositions& JointControllerStarlETHWithSEA::getJointPositions() const {
  return desJointPositions_;
}
const JointVelocities& JointControllerStarlETHWithSEA::getJointVelocities() const {
  return desJointVelocities_;
}

void JointControllerStarlETHWithSEA::setJointControlGainsHAA(double kp,
                                                             double kd) {
  jointPositionControlProportionalGains_(0) = kp;
  jointPositionControlProportionalGains_(3) = kp;
  jointPositionControlProportionalGains_(6) = kp;
  jointPositionControlProportionalGains_(9) = kp;
  jointPositionControlDerivativeGains_(0) = kd;
  jointPositionControlDerivativeGains_(3) = kd;
  jointPositionControlDerivativeGains_(6) = kd;
  jointPositionControlDerivativeGains_(9) = kd;

  jointVelocityControlProportionalGains_(0) = kp;
  jointVelocityControlProportionalGains_(3) = kp;
  jointVelocityControlProportionalGains_(6) = kp;
  jointVelocityControlProportionalGains_(9) = kp;
  jointVelocityControlDerivativeGains_(0) = kd;
  jointVelocityControlDerivativeGains_(3) = kd;
  jointVelocityControlDerivativeGains_(6) = kd;
  jointVelocityControlDerivativeGains_(9) = kd;
}

void JointControllerStarlETHWithSEA::setJointControlGainsHFE(double kp,
                                                             double kd) {
  jointPositionControlProportionalGains_(1) = kp;
  jointPositionControlProportionalGains_(4) = kp;
  jointPositionControlProportionalGains_(7) = kp;
  jointPositionControlProportionalGains_(10) = kp;
  jointPositionControlDerivativeGains_(1) = kd;
  jointPositionControlDerivativeGains_(4) = kd;
  jointPositionControlDerivativeGains_(7) = kd;
  jointPositionControlDerivativeGains_(10) = kd;

  jointVelocityControlProportionalGains_(1) = kp;
  jointVelocityControlProportionalGains_(4) = kp;
  jointVelocityControlProportionalGains_(7) = kp;
  jointVelocityControlProportionalGains_(10) = kp;
  jointVelocityControlDerivativeGains_(1) = kd;
  jointVelocityControlDerivativeGains_(4) = kd;
  jointVelocityControlDerivativeGains_(7) = kd;
  jointVelocityControlDerivativeGains_(10) = kd;
}

void JointControllerStarlETHWithSEA::setJointControlGainsKFE(double kp,
                                                             double kd) {
  jointPositionControlProportionalGains_(2) = kp;
  jointPositionControlProportionalGains_(5) = kp;
  jointPositionControlProportionalGains_(8) = kp;
  jointPositionControlProportionalGains_(11) = kp;
  jointPositionControlDerivativeGains_(2) = kd;
  jointPositionControlDerivativeGains_(5) = kd;
  jointPositionControlDerivativeGains_(8) = kd;
  jointPositionControlDerivativeGains_(11) = kd;

  jointVelocityControlProportionalGains_(2) = kp;
  jointVelocityControlProportionalGains_(5) = kp;
  jointVelocityControlProportionalGains_(8) = kp;
  jointVelocityControlProportionalGains_(11) = kp;
  jointVelocityControlDerivativeGains_(2) = kd;
  jointVelocityControlDerivativeGains_(5) = kd;
  jointVelocityControlDerivativeGains_(8) = kd;
  jointVelocityControlDerivativeGains_(11) = kd;
}

void JointControllerStarlETHWithSEA::setMaxTorqueHAA(double maxTorque) {
  jointMaxTorques_(0) = maxTorque;
  jointMaxTorques_(3) = maxTorque;
  jointMaxTorques_(6) = maxTorque;
  jointMaxTorques_(9) = maxTorque;
}

void JointControllerStarlETHWithSEA::setMaxTorqueHFE(double maxTorque) {
  jointMaxTorques_(1) = maxTorque;
  jointMaxTorques_(4) = maxTorque;
  jointMaxTorques_(7) = maxTorque;
  jointMaxTorques_(10) = maxTorque;
}

void JointControllerStarlETHWithSEA::setMaxTorqueKFE(double maxTorque) {
  jointMaxTorques_(2) = maxTorque;
  jointMaxTorques_(5) = maxTorque;
  jointMaxTorques_(8) = maxTorque;
  jointMaxTorques_(11) = maxTorque;
}

void JointControllerStarlETHWithSEA::setMinTorqueHAA(double minTorque) {
  jointMinTorques_(0) = minTorque;
  jointMinTorques_(3) = minTorque;
  jointMinTorques_(6) = minTorque;
  jointMinTorques_(9) = minTorque;
}

void JointControllerStarlETHWithSEA::setMinTorqueHFE(double minTorque) {
  jointMinTorques_(1) = minTorque;
  jointMinTorques_(4) = minTorque;
  jointMinTorques_(7) = minTorque;
  jointMinTorques_(10) = minTorque;
}

void JointControllerStarlETHWithSEA::setMinTorqueKFE(double minTorque) {
  jointMinTorques_(2) = minTorque;
  jointMinTorques_(5) = minTorque;
  jointMinTorques_(8) = minTorque;
  jointMinTorques_(11) = minTorque;
}

bool JointControllerStarlETHWithSEA::loadParameters(const TiXmlHandle& handle) {
  double kp, kd, maxTorque;

  TiXmlElement* pElem;
  pElem = handle.FirstChild("JointController").FirstChild("StarlETH").Element();
  if (!pElem) {
    printf("Could not find JointController:StarlETH\n");
    return false;
  }

  TiXmlHandle hJointController(
      handle.FirstChild("JointController").FirstChild("StarlETH"));

  pElem = hJointController.FirstChild("HAA").Element();
  if (!pElem) {
    printf("Could not find JointController:StarlETH:HAA\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("kp", &kp) != TIXML_SUCCESS) {
    printf("Could not find HAA:kp\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("kd", &kd) != TIXML_SUCCESS) {
    printf("Could not find HAA:kd\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("maxTorque", &maxTorque) != TIXML_SUCCESS) {
    printf("Could not find HAA:maxTorque\n");
    return false;
  }
  setJointControlGainsHAA(kp, kd);
  setMaxTorqueHAA(maxTorque);

  /* HFE */
  pElem = hJointController.FirstChild("HFE").Element();
  if (!pElem) {
    printf("Could not find JointController:StarlETH:HFE\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("kp", &kp) != TIXML_SUCCESS) {
    printf("Could not find HFE:kp\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("kd", &kd) != TIXML_SUCCESS) {
    printf("Could not find HFE:kd\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("maxTorque", &maxTorque) != TIXML_SUCCESS) {
    printf("Could not find HFE:maxTorque\n");
    return false;
  }
  setJointControlGainsHFE(kp, kd);
  setMaxTorqueHFE(maxTorque);

  /* KFE */
  pElem = hJointController.FirstChild("KFE").Element();
  if (!pElem) {
    printf("Could not find JointController:StarlETH:KFE\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("kp", &kp) != TIXML_SUCCESS) {
    printf("Could not find KFE:kp\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("kd", &kd) != TIXML_SUCCESS) {
    printf("Could not find KFE:kd\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("maxTorque", &maxTorque) != TIXML_SUCCESS) {
    printf("Could not find KFE:maxTorque\n");
    return false;
  }
  setJointControlGainsKFE(kp, kd);
  setMaxTorqueKFE(maxTorque);

  return true;
}

std::ostream& operator <<(std::ostream& out,
                          const JointControllerStarlETHWithSEA& controller) {

  out << (
      controller.isClampingPositions_ ?
          "Joint positions are limited.\n" :
          "Joint positions are NOT limited.\n");
  out << (
      controller.isClampingVelocities_ ?
          "Joint velocities are limited.\n" :
          "Joint velocities are NOT limited.\n");
  out << (
      controller.isClampingTorques_ ?
          "Joint torques are limited.\n" : "Joint torques are NOT limited.\n");

  out << "\nJoint position limits [min, max] rad: \n";
  out << "LF_HAA: [" << controller.jointMinPositions_(0) << ", "
      << controller.jointMaxPositions_(0) << "]\n";
  out << "LF_HFE: [" << controller.jointMinPositions_(1) << ", "
      << controller.jointMaxPositions_(1) << "]\n";
  out << "LF_KFE: [" << controller.jointMinPositions_(2) << ", "
      << controller.jointMaxPositions_(2) << "]\n";
  out << "RF_HAA: [" << controller.jointMinPositions_(3) << ", "
      << controller.jointMaxPositions_(3) << "]\n";
  out << "RF_HFE: [" << controller.jointMinPositions_(4) << ", "
      << controller.jointMaxPositions_(4) << "]\n";
  out << "RF_KFE: [" << controller.jointMinPositions_(5) << ", "
      << controller.jointMaxPositions_(5) << "]\n";
  out << "LH_HAA: [" << controller.jointMinPositions_(6) << ", "
      << controller.jointMaxPositions_(6) << "]\n";
  out << "LH_HFE: [" << controller.jointMinPositions_(7) << ", "
      << controller.jointMaxPositions_(7) << "]\n";
  out << "LH_KFE: [" << controller.jointMinPositions_(8) << ", "
      << controller.jointMaxPositions_(8) << "]\n";
  out << "RH_HAA: [" << controller.jointMinPositions_(9) << ", "
      << controller.jointMaxPositions_(9) << "]\n";
  out << "RH_HFE: [" << controller.jointMinPositions_(10) << ", "
      << controller.jointMaxPositions_(10) << "]\n";
  out << "RH_KFE: [" << controller.jointMinPositions_(11) << ", "
      << controller.jointMaxPositions_(11) << "]\n";

  out << "\nJoint velocity limits: [min, max] rad/s\n";
  out << "LF_HAA: [" << controller.jointMinVelocities_(0) << ", "
      << controller.jointMaxVelocities_(0) << "]\n";
  out << "LF_HFE: [" << controller.jointMinVelocities_(1) << ", "
      << controller.jointMaxVelocities_(1) << "]\n";
  out << "LF_KFE: [" << controller.jointMinVelocities_(2) << ", "
      << controller.jointMaxVelocities_(2) << "]\n";
  out << "RF_HAA: [" << controller.jointMinVelocities_(3) << ", "
      << controller.jointMaxVelocities_(3) << "]\n";
  out << "RF_HFE: [" << controller.jointMinVelocities_(4) << ", "
      << controller.jointMaxVelocities_(4) << "]\n";
  out << "RF_KFE: [" << controller.jointMinVelocities_(5) << ", "
      << controller.jointMaxVelocities_(5) << "]\n";
  out << "LH_HAA: [" << controller.jointMinVelocities_(6) << ", "
      << controller.jointMaxVelocities_(6) << "]\n";
  out << "LH_HFE: [" << controller.jointMinVelocities_(7) << ", "
      << controller.jointMaxVelocities_(7) << "]\n";
  out << "LH_KFE: [" << controller.jointMinVelocities_(8) << ", "
      << controller.jointMaxVelocities_(8) << "]\n";
  out << "RH_HAA: [" << controller.jointMinVelocities_(9) << ", "
      << controller.jointMaxVelocities_(9) << "]\n";
  out << "RH_HFE: [" << controller.jointMinVelocities_(10) << ", "
      << controller.jointMaxVelocities_(10) << "]\n";
  out << "RH_KFE: [" << controller.jointMinVelocities_(11) << ", "
      << controller.jointMaxVelocities_(11) << "]\n";

  out << "\nJoint torque limits: [min, max] Nm\n";
  out << "LF_HAA: [" << controller.jointMinTorques_(0) << ", "
      << controller.jointMaxTorques_(0) << "]\n";
  out << "LF_HFE: [" << controller.jointMinTorques_(1) << ", "
      << controller.jointMaxTorques_(1) << "]\n";
  out << "LF_KFE: [" << controller.jointMinTorques_(2) << ", "
      << controller.jointMaxTorques_(2) << "]\n";
  out << "RF_HAA: [" << controller.jointMinTorques_(3) << ", "
      << controller.jointMaxTorques_(3) << "]\n";
  out << "RF_HFE: [" << controller.jointMinTorques_(4) << ", "
      << controller.jointMaxTorques_(4) << "]\n";
  out << "RF_KFE: [" << controller.jointMinTorques_(5) << ", "
      << controller.jointMaxTorques_(5) << "]\n";
  out << "LH_HAA: [" << controller.jointMinTorques_(6) << ", "
      << controller.jointMaxTorques_(6) << "]\n";
  out << "LH_HFE: [" << controller.jointMinTorques_(7) << ", "
      << controller.jointMaxTorques_(7) << "]\n";
  out << "LH_KFE: [" << controller.jointMinTorques_(8) << ", "
      << controller.jointMaxTorques_(8) << "]\n";
  out << "RH_HAA: [" << controller.jointMinTorques_(9) << ", "
      << controller.jointMaxTorques_(9) << "]\n";
  out << "RH_HFE: [" << controller.jointMinTorques_(10) << ", "
      << controller.jointMaxTorques_(10) << "]\n";
  out << "RH_KFE: [" << controller.jointMinTorques_(11) << ", "
      << controller.jointMaxTorques_(11) << "]\n";

  out << "\nPosition-control gains: [p-gain, d-gain]\n";
  out << "LF_HAA: [" << controller.jointPositionControlProportionalGains_(0)
      << ", " << controller.jointPositionControlDerivativeGains_(0) << "]\n";
  out << "LF_HFE: [" << controller.jointPositionControlProportionalGains_(1)
      << ", " << controller.jointPositionControlDerivativeGains_(1) << "]\n";
  out << "LF_KFE: [" << controller.jointPositionControlProportionalGains_(2)
      << ", " << controller.jointPositionControlDerivativeGains_(2) << "]\n";
  out << "RF_HAA: [" << controller.jointPositionControlProportionalGains_(3)
      << ", " << controller.jointPositionControlDerivativeGains_(3) << "]\n";
  out << "RF_HFE: [" << controller.jointPositionControlProportionalGains_(4)
      << ", " << controller.jointPositionControlDerivativeGains_(4) << "]\n";
  out << "RF_KFE: [" << controller.jointPositionControlProportionalGains_(5)
      << ", " << controller.jointPositionControlDerivativeGains_(5) << "]\n";
  out << "LH_HAA: [" << controller.jointPositionControlProportionalGains_(6)
      << ", " << controller.jointPositionControlDerivativeGains_(6) << "]\n";
  out << "LH_HFE: [" << controller.jointPositionControlProportionalGains_(7)
      << ", " << controller.jointPositionControlDerivativeGains_(7) << "]\n";
  out << "LH_KFE: [" << controller.jointPositionControlProportionalGains_(8)
      << ", " << controller.jointPositionControlDerivativeGains_(8) << "]\n";
  out << "RH_HAA: [" << controller.jointPositionControlProportionalGains_(9)
      << ", " << controller.jointPositionControlDerivativeGains_(9) << "]\n";
  out << "RH_HFE: [" << controller.jointPositionControlProportionalGains_(10)
      << ", " << controller.jointPositionControlDerivativeGains_(10) << "]\n";
  out << "RH_KFE: [" << controller.jointPositionControlProportionalGains_(11)
      << ", " << controller.jointPositionControlDerivativeGains_(11) << "]\n";

  out << "\nVelocity-control gains: [p-gain, d-gain]\n";
  out << "LF_HAA: [" << controller.jointVelocityControlProportionalGains_(0)
      << ", " << controller.jointVelocityControlDerivativeGains_(0) << "]\n";
  out << "LF_HFE: [" << controller.jointVelocityControlProportionalGains_(1)
      << ", " << controller.jointVelocityControlDerivativeGains_(1) << "]\n";
  out << "LF_KFE: [" << controller.jointVelocityControlProportionalGains_(2)
      << ", " << controller.jointVelocityControlDerivativeGains_(2) << "]\n";
  out << "RF_HAA: [" << controller.jointVelocityControlProportionalGains_(3)
      << ", " << controller.jointVelocityControlDerivativeGains_(3) << "]\n";
  out << "RF_HFE: [" << controller.jointVelocityControlProportionalGains_(4)
      << ", " << controller.jointVelocityControlDerivativeGains_(4) << "]\n";
  out << "RF_KFE: [" << controller.jointVelocityControlProportionalGains_(5)
      << ", " << controller.jointVelocityControlDerivativeGains_(5) << "]\n";
  out << "LH_HAA: [" << controller.jointVelocityControlProportionalGains_(6)
      << ", " << controller.jointVelocityControlDerivativeGains_(6) << "]\n";
  out << "LH_HFE: [" << controller.jointVelocityControlProportionalGains_(7)
      << ", " << controller.jointVelocityControlDerivativeGains_(7) << "]\n";
  out << "LH_KFE: [" << controller.jointVelocityControlProportionalGains_(8)
      << ", " << controller.jointVelocityControlDerivativeGains_(8) << "]\n";
  out << "RH_HAA: [" << controller.jointVelocityControlProportionalGains_(9)
      << ", " << controller.jointVelocityControlDerivativeGains_(9) << "]\n";
  out << "RH_HFE: [" << controller.jointVelocityControlProportionalGains_(10)
      << ", " << controller.jointVelocityControlDerivativeGains_(10) << "]\n";
  out << "RH_KFE: [" << controller.jointVelocityControlProportionalGains_(11)
      << ", " << controller.jointVelocityControlDerivativeGains_(11) << "]\n";

  return out;
}

} /* namespace loco */

