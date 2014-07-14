/*
 * MotorDynamics.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: wko
 */

#include "MotorDynamics.hpp"

MotorDynamics::MotorDynamics()
    : absMaxVel_(10.0),
      absMaxPow_(180.0),
      isMotorPositionsInitialized_(false) {

}

MotorDynamics::~MotorDynamics() {

}

/**
 * Low-pass filter for desired velocity.
 */
double MotorDynamics::lowPass(int index, double desMotorVelocity, double cutOff,
                              double dt) {
  double a0, b0;

  a0 = dt / (1 / cutOff + dt);
  b0 = (1 / cutOff) / (1 / cutOff + dt);

  return desMotorVelocity * b0 + measuredMotorVelocities_(index) * a0;
}

bool MotorDynamics::initialize(loco::JointVelocities minVel,
                               loco::JointVelocities maxVel) {
  measuredMotorVelocities_.setZero();

  jointMinVelocities_ = minVel;
  jointMaxVelocities_ = maxVel;

  return true;
}

double MotorDynamics::getMeasuredMotorPosition(int jointIndex) {
  return measuredMotorPositions_(jointIndex);
}

/**
 * Only for initializing the measuredMotorPositions.
 */
bool MotorDynamics::initializeMotorPositions(
    loco::JointPositions motorPositions) {

  if (!isMotorPositionsInitialized_) {
    measuredMotorPositions_ = motorPositions;
  } else {
    std::cout << "Already initialized!" << std::endl;
    return false;
  }
  return true;
}

/**
 * Transfer function for motor dynamics (from desired motor velocity to measured motor velocity).
 * Tracks a given reference motor velocity.
 * @param index: index of current joint
 * @param dt: time step
 * @param desMotorVelocity: velocity reference for current joint
 */
double MotorDynamics::trackMotorVelocity(int jointIndex,
                                         double desMotorVelocity,
                                         double previousJointTorque,
                                         double dt) {

  /* Without low pass */
//  double measuredMotorVelocity = desMotorVelocity_(i);
  desMotorVelocity = lowPass(jointIndex, desMotorVelocity, 100, dt);

  double maxVelocity;

  // Set max velocity to absolute max velocity (physical limit) if previous torque is zero.
  // Otherwise, calculate max velocity by using max power
  maxVelocity =
      previousJointTorque == 0.0 ?
          absMaxVel_ : absMaxPow_ / std::abs(previousJointTorque);

  // Clamp velocity according to power limit.
  if (desMotorVelocity > maxVelocity) {
    desMotorVelocity = maxVelocity;
  } else if (desMotorVelocity < -maxVelocity) {
    desMotorVelocity = -maxVelocity;
  }

  // Make sure the velocity is still within bounds of physical limits.
  if (desMotorVelocity > jointMaxVelocities_(jointIndex)) {
    desMotorVelocity = jointMaxVelocities_(jointIndex);
  } else if (desMotorVelocity < jointMinVelocities_(jointIndex)) {
    desMotorVelocity = jointMinVelocities_(jointIndex);
  }

  measuredMotorPositions_(jointIndex) += desMotorVelocity * dt;

  return desMotorVelocity;
}

