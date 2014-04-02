/*
 * JointControllerStarlETH.cpp
 *
 *  Created on: Apr 2, 2014
 *      Author: gech
 */

#include "loco/joint_control/JointControllerStarlETH.hpp"

namespace loco {

JointControllerStarlETH::JointControllerStarlETH(robotModel::RobotModel* robotModel) :
 JointControllerBase(),
 robotModel_(robotModel)
{

}

JointControllerStarlETH::~JointControllerStarlETH() {

}


bool JointControllerStarlETH::initialize(double dt) {
  desJointModesPrevious_ = robotModel_->act().getMode();
  desPositionsInVelocityControl_ = robotModel_->q().getQj(); //robotModel_->act().getPos();
  jointTorques_.setZero();
  return true;
}

bool JointControllerStarlETH::advance(double dt) {

  const robotModel::VectorActM desJointModes = robotModel_->act().getMode();
  const robotModel::VectorAct desJointPositions = robotModel_->act().getPos();
  const robotModel::VectorAct desJointVelocities = robotModel_->act().getVel();
  const robotModel::VectorAct desJointTorques = robotModel_->act().getTau();

  const robotModel::VectorQj measJointPositions = robotModel_->q().getQj();
  const robotModel::VectorQj measJointVelocities = robotModel_->q().getdQj();


  for (int i=0; i<desJointModes.size(); i++) {
    if (desJointModes(i) == robotModel::AM_Position) {
      jointTorques_(i) = jointPositionControlProportionalGains_(i)*(desJointPositions(i)-measJointPositions(i));
      jointTorques_(i) += jointPositionControlDerivativeGains_(i)*(desJointVelocities(i)-measJointVelocities(i));
    }
    else if(desJointModes(i) == robotModel::AM_Velocity) {

      if(desJointModes(i) != desJointModesPrevious_(i)) {
        desPositionsInVelocityControl_(i) = desJointPositions(i);
      }
      jointTorques_(i) = jointVelocityControlProportionalGains_(i)*(desPositionsInVelocityControl_(i)-measJointPositions(i));
      jointTorques_(i) += jointVelocityControlDerivativeGains_(i)*(desJointVelocities(i)-measJointVelocities(i));
    }
    else if(desJointModes(i) == robotModel::AM_Torque) {
      jointTorques_(i) = desJointTorques(i);
    }
    else {
     throw "Joint control mode is not supported!";
    }
    std::cout << "joint mode " << i << " :" << (int) desJointModes(i) << " pos: " << desJointPositions(i) << " vel:"  << desJointVelocities(i) << " tau: " << jointTorques_(i) << std::endl;
  }

  desJointModesPrevious_ = desJointModes;


  return true;
}

const JointControllerStarlETH::JointTorques& JointControllerStarlETH::getJointTorques() const {
  return jointTorques_;
}

void JointControllerStarlETH::setJointControlGainsHAA(double kp,
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

void JointControllerStarlETH::setJointControlGainsHFE(double kp,
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

void JointControllerStarlETH::setJointControlGainsKFE(double kp,
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

} /* namespace loco */


