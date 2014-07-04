/*
 * JointControllerStarlETHWithSEA.cpp
 *
 *  Created on: Apr 2, 2014
 *      Author: gech
 */

#include "loco/joint_control/JointControllerStarlETHWithSEA.hpp"
#include <limits>
namespace loco {

JointControllerStarlETHWithSEA::JointControllerStarlETHWithSEA(
    robotModel::RobotModel* robotModel)
    : robotModel_(robotModel),
      isClampingTorques_(false),
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
  const double deltaLimit = DELTA_LIMIT;  // [rad]
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
  const double absMaxTorque = ABSOLUTE_MAX_TORQUE;  // [Nm]
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
  const double absMaxVelocity = ABSOLUTE_MAX_VELOCITY;  // [m/s]
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
  //  LF_LEG position limits
  jointMinPositions_(0) = jointMinPositionsForLegInDefaultConfiguration(0);
  jointMaxPositions_(0) = jointMaxPositionsForLegInDefaultConfiguration(0);

  //  RF_LEG position limits
  jointMinPositions_(3) = -jointMaxPositionsForLegInDefaultConfiguration(0);
  jointMaxPositions_(3) = -jointMinPositionsForLegInDefaultConfiguration(0);

  //  LH_LEG position limits
  jointMinPositions_(6) = jointMinPositionsForLegInDefaultConfiguration(0);
  jointMaxPositions_(6) = jointMaxPositionsForLegInDefaultConfiguration(0);

  //  RH_LEG position limits
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

}

/**
 * Ensures that a number is clamped between a lower bound and an upper bound.
 * @param number: Number to be clamped.
 * @param lower: Lower bound.
 * @param upper: Upper bound.
 */
inline double clamp(double number, double lower, double upper) {
  return std::max(lower, std::min(number, upper));
}

/**
 * Transfer function from desired torque to measured torque.
 * Tracks a given reference joint torque
 * @param dt: time step
 */
bool JointControllerStarlETHWithSEA::trackMotorTorques(double dt) {
  const robotModel::VectorAct desJointTorques = robotModel_->act().getTau();

  for (int i = 0; i < desJointTorques.size(); i++) {
    desJointTorques_(i) = desJointTorques(i);
    desiredMotorVelocities_(i) = pGains_(i)
        * (desJointTorques_(i) - previousMeasuredJointTorques_(i));
  }

  return trackMotorVelocities(dt);
}

/**
 * Transfer function for motor dynamics (from desired motor velocity to measured motor velocity).
 * Tracks a given reference motor velocity
 * @param dt: time step
 */
bool JointControllerStarlETHWithSEA::trackMotorVelocities(double dt) {

  measuredMotorVelocities_ = desiredMotorVelocities_;

  JointVelocities maxVelocities, minVelocities;

  // Just fill evertyhing with absolute max velocity if all torques are zero
  if (previousMeasuredJointTorques_.sum() == 0.0) {
    maxVelocities = jointMaxVelocities_;
    minVelocities = jointMinVelocities_;
  } else {
    // Otherwise, calculate max velocity by using max power, but still use absolute max velocity if torque is zero.
    for (int i = 0; i < TOTAL_NUMBER_OF_JOINTS; i++) {
      maxVelocities(i) =
          previousMeasuredJointTorques_(i) == 0.0 ?
              ABSOLUTE_MAX_VELOCITY :
              MAX_POWER / previousMeasuredJointTorques_(i);
    }
    minVelocities = -maxVelocities;
  }

  for (int i = 0; i < TOTAL_NUMBER_OF_JOINTS; i++) {
    clamp((double) desiredMotorVelocities_(i), (double) jointMinVelocities_(i),
          (double) jointMaxVelocities_(i));

    clamp((double) desiredMotorVelocities_(i), (double) minVelocities(i),
          (double) maxVelocities(i));
  }

  measuredMotorVelocities_ = desiredMotorVelocities_;
  return transferFunctionMotorVelocitiesToTorques(dt);
}

/**
 * Transfer function from motor velocity to joint torque.
 * Closes the loop for torque control.
 * @param dt: time step
 */
bool JointControllerStarlETHWithSEA::transferFunctionMotorVelocitiesToTorques(
    double dt) {

  robotModel::VectorQj motorPositionsToSet;
  robotModel::VectorAct jointTorquesToSet;

  for (int i = 0; i < TOTAL_NUMBER_OF_JOINTS; i++) {
    measuredMotorPositions_(i) += measuredMotorVelocities_(i) * dt;
    motorPositionsToSet(i) = measuredMotorPositions_(i);

    measuredJointVelocities_(i) = robotModel_->q().getdQj()(i);
    measuredJointPositions_(i) = robotModel_->q().getQj()(i);
    previousMeasuredJointTorques_(i) = measuredJointTorques_(i);

    measuredJointTorques_(i) = springStiffnesses_(i)
        * (measuredMotorPositions_(i) - measuredJointPositions_(i))
        + springDampings_(i)
            * (measuredMotorVelocities_(i) - measuredJointVelocities_(i));

    jointTorquesToSet(i) = measuredJointTorques_(i);
  }

  robotModel_->sensors().setMotorPos(motorPositionsToSet);

  robotModel_->sensors().setJointTorques(jointTorquesToSet);
  robotModel_->act().setTau(jointTorquesToSet);

  return true;
}

const JointTorques& JointControllerStarlETHWithSEA::getJointTorques() const {
  return jointTorques_;
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

  // We start in the air, so torques are zero.
  desJointTorques_.setZero();

  robotModel::VectorQj motorPositionsToSet;
  robotModel::VectorAct measuredJointTorques = robotModel_->sensors()
      .getJointTorques();

  for (int i = 0; i < TOTAL_NUMBER_OF_JOINTS; i++) {
    previousMeasuredJointTorques_(i) = measuredJointTorques(i);

    // Set motor positions to joint positions for initialization
    measuredJointPositions_(i) = robotModel_->q().getQj()(i);
    motorPositionsToSet(i) = measuredJointPositions_(i);
  }

  robotModel_->sensors().setMotorPos(motorPositionsToSet);

  // Ensure that motor velocities are zero
  measuredMotorVelocities_.setZero();
  robotModel::VectorQj motorVelocitiesToSet;
  motorVelocitiesToSet.fill(0);

  robotModel_->sensors().setMotorVel(motorVelocitiesToSet);

  // Put spring parameters in vectors
  for (int i = 0; i < TOTAL_NUMBER_OF_JOINTS; i +=
      (int) JointTypes::NUMBER_OF_JOINT_TYPES) {

    springStiffnesses_[JointTypes::HAA + i] = robotModel_->params()
        .springStiffnesses_.haa;
    springStiffnesses_[JointTypes::HFE + i] = robotModel_->params()
        .springStiffnesses_.hfe;
    springStiffnesses_[JointTypes::KFE + i] = robotModel_->params()
        .springStiffnesses_.kfe;

    springDampings_[JointTypes::HAA + i] = robotModel_->params()
        .springDampings_.haa;
    springDampings_[JointTypes::HFE + i] = robotModel_->params()
        .springDampings_.hfe;
    springDampings_[JointTypes::KFE + i] = robotModel_->params()
        .springDampings_.kfe;

    // Set P-gains
    pGains_[JointTypes::HAA + i] = PGAIN_HAA;
    pGains_[JointTypes::HFE + i] = PGAIN_HFE;
    pGains_[JointTypes::KFE + i] = PGAIN_KFE;
  }

  desJointModesPrevious_ = robotModel_->act().getMode();
  desPositionsInVelocityControl_ = robotModel_->q().getQj();  //robotModel_->act().getPos();
  jointTorques_.setZero();
  desJointModesPrevious_.fill(robotModel::AM_Velocity);

  return true;
}

bool JointControllerStarlETHWithSEA::advance(double dt) {
/*
   if (!trackMotorTorques(dt)) {
   std::cout << "Error while tracking torques!" << std::endl;

   return false;
   }
*/

  const robotModel::VectorActM desJointModes = robotModel_->act().getMode();
  jointPositions_ = JointPositions(robotModel_->act().getPos());
  jointVelocities_ = JointVelocities(robotModel_->act().getVel());
  const robotModel::VectorAct desJointTorques = robotModel_->act().getTau();

  const robotModel::VectorQj measJointPositions = robotModel_->q().getQj();
  const robotModel::VectorQj measJointVelocities = robotModel_->q().getdQj();

  for (int i = 0; i < desJointModes.size(); i++) {
    if (desJointModes(i) == robotModel::AM_Position) {
      if (isClampingPositions_) {
        if (jointPositions_(i) > jointMaxPositions_(i)) {
          jointPositions_(i) = jointMaxPositions_(i);
        } else if (jointPositions_(i) < jointMinPositions_(i)) {
          jointPositions_(i) = jointMinPositions_(i);
        }
      }
      jointTorques_(i) = jointPositionControlProportionalGains_(i)
          * (jointPositions_(i) - measJointPositions(i));
      jointTorques_(i) += jointPositionControlDerivativeGains_(i)
          * (jointVelocities_(i) - measJointVelocities(i));

    } else if (desJointModes(i) == robotModel::AM_Velocity) {
      if (isClampingVelocities_) {
        if (jointVelocities_(i) > jointMaxVelocities_(i)) {
          jointVelocities_(i) = jointMaxVelocities_(i);
        } else if (jointVelocities_(i) < jointMinVelocities_(i)) {
          jointVelocities_(i) = jointMinVelocities_(i);
        }
      }
      if (desJointModes(i) != desJointModesPrevious_(i)) {
        desPositionsInVelocityControl_(i) = jointPositions_(i);
      }
      jointTorques_(i) = jointVelocityControlProportionalGains_(i)
          * (desPositionsInVelocityControl_(i) - measJointPositions(i));
      jointTorques_(i) += jointVelocityControlDerivativeGains_(i)
          * (jointVelocities_(i) - measJointVelocities(i));

    } else if (desJointModes(i) == robotModel::AM_Torque) {
      jointTorques_(i) = desJointTorques(i);
    } else {
      throw "Joint control mode is not supported!";
    }
//    std::cout << "joint mode " << i << " :" << (int) desJointModes(i) << " pos: " << desJointPositions(i) << " vel:"  << desJointVelocities(i) << " tau: " << jointTorques_(i) << std::endl;
    if (isClampingTorques_) {
      if (jointTorques_(i) > jointMaxTorques_(i)) {
        jointTorques_(i) = jointMaxTorques_(i);
      } else if (jointTorques_(i) < jointMinTorques_(i)) {
        jointTorques_(i) = jointMinTorques_(i);
      }
    }
  }

  desJointModesPrevious_ = desJointModes;

  return true;

//  return trackMotorTorques(dt);
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

} /* namespace loco */

