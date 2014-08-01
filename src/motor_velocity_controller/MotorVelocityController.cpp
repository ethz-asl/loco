/*
 * MotorVelocityController.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: labstudent
 */

#include "loco/motor_velocity_controller/MotorVelocityController.hpp"

namespace loco {

MotorVelocityController::MotorVelocityController(
    robotModel::RobotModel *robotModel, LegGroup* legs, TorsoBase* torso) {
  robotModel_ = robotModel;
  legs_ = legs;
  torso_ = torso;
}

MotorVelocityController::~MotorVelocityController() {

}

// No parameters for now
bool MotorVelocityController::loadParameters(const TiXmlHandle &handle) {

  return true;
}

bool MotorVelocityController::initialize(double dt) {

  // Make sure the robot is on the ground and did not jump before
  // If true, the robot is in INIT state
  if (legs_->getLeftForeLeg()->isGrounded()
      && legs_->getRightForeLeg()->isGrounded()
      && legs_->getLeftHindLeg()->isGrounded()
      && legs_->getRightForeLeg()->isGrounded() && state_ != State::APEX) {
    state_ = State::INIT;
  }

  LegBase* leftForeLeg = legs_->getLeftForeLeg();
  leftForeInitJointPositions_ = leftForeLeg->getMeasuredJointPositions();

  input_vel.open("vel_input_kfe");
  output_vel.open("vel_input_hfe");
//  for (int i = 0; i < leftForeInitJointPositions_.size(); i++) {
//    std::cout << leftForeInitJointPositions_[i] << std::endl;
//  }

  return true;
}

/**
 * Returns phase of execution.
 */
State MotorVelocityController::getState() {
  return state_;
}

/**
 * Sets GaussianKernelJumpPropagator to follow.
 */
void MotorVelocityController::setTrajectoryFollower(
    GaussianKernelJumpPropagator trajectoryFollower) {
  trajectoryFollower_ = trajectoryFollower;
}

/**
 * Enables joint velocity control when set.
 */
void MotorVelocityController::setInVelocityMode(bool isInVelocityMode) {
  inVelocityMode_ = isInVelocityMode;
}

void MotorVelocityController::advance(double dt) {
  Eigen::Vector3d velocities;
  Eigen::VectorXd result;

  static double currentTime = 0;
  if (inVelocityMode_) {
    // Different joint types with indices 0, 1, 2 respectively (in this order).
    updateState();

    if (state_ == LIFTOFF || state_ == State::INIT) {
       result = trajectoryFollower_.predict();

       for (int i = 0; i < velocities.size(); i++) {
         velocities(i) = result(i);
       }

      robotModel_->act().setVelOfLeg(velocities, 0);
      robotModel_->act().setVelOfLeg(velocities, 1);
      robotModel_->act().setVelOfLeg(-velocities, 2);
      robotModel_->act().setVelOfLeg(-velocities, 3);

      output_vel << currentTime << " " << velocities(1) << std::endl;
      input_vel << currentTime << " " << velocities(2) << std::endl;
    } else {
      velocities.fill(0);
      for (int i = 0; i < 4; i++)
        robotModel_->act().setVelOfLeg(velocities, i);
    }
  }
  currentTime += dt;
}

/**
 * Keeps track of current state of jump.
 * Jump occurs in the following phases:
 * INIT -> LIFTOFF -> APEX -> TOUCHDOWN
 */
void MotorVelocityController::updateState() {
  if (legs_->getLeftForeLeg()->isGrounded()
      && legs_->getRightForeLeg()->isGrounded()
      && legs_->getLeftHindLeg()->isGrounded()
      && legs_->getRightHindLeg()->isGrounded()
      && state_ == State::INIT) {

    state_ = State::LIFTOFF;

  } else if (!legs_->getLeftForeLeg()->isGrounded()
      && !legs_->getLeftHindLeg()->isGrounded()
      && !legs_->getRightHindLeg()->isGrounded()
      && !legs_->getRightForeLeg()->isGrounded() && state_ == State::LIFTOFF) {

    state_ = State::APEX;

  } else if (legs_->getLeftForeLeg()->isGrounded()
      && legs_->getRightForeLeg()->isGrounded()
      && legs_->getLeftHindLeg()->isGrounded()
      && legs_->getRightForeLeg()->isGrounded() && state_ == State::APEX) {

    state_ = State::TOUCHDOWN;
  }
}
} /* namespace loco */

