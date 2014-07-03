/*
 * JointControllerStarlETH.cpp
 *
 *  Created on: Apr 2, 2014
 *      Author: gech
 */

#include "loco/joint_control/JointControllerStarlETH.hpp"
#include <limits>
namespace loco {

JointControllerStarlETH::JointControllerStarlETH(robotModel::RobotModel* robotModel) :
 JointControllerBase(),
 robotModel_(robotModel),
 isClampingTorques_(false)
{
  jointMaxTorques_.toImplementation().fill(std::numeric_limits<double>::max());
//  setJointControlGainsHAA(70.0, 3.1);
//  setJointControlGainsHFE(70.0, 3.2);
//  setJointControlGainsKFE(26.0, 0.2);


  /* X-Configuration */
  isLegInDefaultConfiguration_[0] = true;
  isLegInDefaultConfiguration_[1] = true;
  isLegInDefaultConfiguration_[2] = false;
  isLegInDefaultConfiguration_[3] = false;



  const double deltaLimit = 0.05;
  Eigen::Vector3d jointMinPositionsForLegInDefaultConfiguration;
  Eigen::Vector3d jointMaxPositionsForLegInDefaultConfiguration;
  jointMinPositionsForLegInDefaultConfiguration(0) = -0.4+deltaLimit; // -0.2+deltaLimit;
  jointMaxPositionsForLegInDefaultConfiguration(0) = 0.5-deltaLimit;  //0.5
  jointMinPositionsForLegInDefaultConfiguration(1) = -0.6+deltaLimit;
  jointMaxPositionsForLegInDefaultConfiguration(1) = 1.29-deltaLimit; //1.29  1.29-2*deltaLimit;
  jointMinPositionsForLegInDefaultConfiguration(2) = -2.46+deltaLimit; //-2.46
  jointMaxPositionsForLegInDefaultConfiguration(2) = -0.1-deltaLimit; // -0.1


  setJointPositionLimitsFromDefaultConfiguration(jointMinPositionsForLegInDefaultConfiguration, jointMaxPositionsForLegInDefaultConfiguration, isLegInDefaultConfiguration_);


}


void JointControllerStarlETH::setJointPositionLimitsFromDefaultConfiguration(const Eigen::Vector3d& jointMinPositionsForLegInDefaultConfiguration,
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

   for (int iLeg=0; iLeg<4; iLeg++) {
     jointMinPositions_(1+iLeg*3) = isLegInDefaultConfiguration[iLeg] ? jointMinPositionsForLegInDefaultConfiguration(1):  -jointMaxPositionsForLegInDefaultConfiguration(1);
     jointMaxPositions_(1+iLeg*3) = isLegInDefaultConfiguration[iLeg] ? jointMaxPositionsForLegInDefaultConfiguration(1) : -jointMinPositionsForLegInDefaultConfiguration(1);
     jointMinPositions_(2+iLeg*3) = isLegInDefaultConfiguration[iLeg] ? jointMinPositionsForLegInDefaultConfiguration(2) : -jointMaxPositionsForLegInDefaultConfiguration(2);
     jointMaxPositions_(2+iLeg*3) = isLegInDefaultConfiguration[iLeg] ? jointMaxPositionsForLegInDefaultConfiguration(2) : -jointMinPositionsForLegInDefaultConfiguration(2);
   }

}
const JointControllerStarlETH::JointVector& JointControllerStarlETH::getJointMaxPositions() const {
  return jointMaxPositions_;
}
const JointControllerStarlETH::JointVector& JointControllerStarlETH::getJointMinPositions() const {
  return jointMinPositions_;
}

JointControllerStarlETH::~JointControllerStarlETH() {

}

void JointControllerStarlETH::setDesiredJointPositionsInVelocityControl(const robotModel::VectorAct& positions) {
  desPositionsInVelocityControl_ = positions;
}

void JointControllerStarlETH::setIsClampingTorques(bool isClamping)  {
  isClampingTorques_ = isClamping;
}

bool JointControllerStarlETH::initialize(double dt) {
  desJointModesPrevious_ = robotModel_->act().getMode();
  desPositionsInVelocityControl_ = robotModel_->q().getQj(); //robotModel_->act().getPos();
  jointTorques_.setZero();
  desJointModesPrevious_.fill(robotModel::AM_Velocity);
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
//    std::cout << "joint mode " << i << " :" << (int) desJointModes(i) << " pos: " << desJointPositions(i) << " vel:"  << desJointVelocities(i) << " tau: " << jointTorques_(i) << std::endl;
    if (isClampingTorques_) {
      const double maxTorque = jointMaxTorques_(i);
      if (jointTorques_(i) > maxTorque) {
        jointTorques_(i) = maxTorque;
      }
      else if (jointTorques_(i) < -maxTorque) {
        jointTorques_(i) = -maxTorque;
      }
    }
  }

  desJointModesPrevious_ = desJointModes;


  return true;
}

const JointTorques& JointControllerStarlETH::getJointTorques() const {
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

void JointControllerStarlETH::setMaxTorqueHAA(double maxTorque) {
  jointMaxTorques_(0) = maxTorque;
  jointMaxTorques_(3) = maxTorque;
  jointMaxTorques_(6) = maxTorque;
  jointMaxTorques_(9) = maxTorque;
}

void JointControllerStarlETH::setMaxTorqueHFE(double maxTorque) {
  jointMaxTorques_(1) = maxTorque;
  jointMaxTorques_(4) = maxTorque;
  jointMaxTorques_(7) = maxTorque;
  jointMaxTorques_(10) = maxTorque;
}

void JointControllerStarlETH::setMaxTorqueKFE(double maxTorque) {
  jointMaxTorques_(2) = maxTorque;
  jointMaxTorques_(5) = maxTorque;
  jointMaxTorques_(8) = maxTorque;
  jointMaxTorques_(11) = maxTorque;
}


bool JointControllerStarlETH::loadParameters(const TiXmlHandle& handle) {
  double kp, kd, maxTorque;

  TiXmlElement* pElem;
  pElem = handle.FirstChild("JointController").FirstChild("StarlETH").Element();
  if (!pElem) {
    printf("Could not find JointController:StarlETH\n");
    return false;
  }
  TiXmlHandle hJointController(handle.FirstChild("JointController").FirstChild("StarlETH"));

  pElem = hJointController.FirstChild("HAA").Element();
  if (!pElem) {
    printf("Could not find JointController:StarlETH:HAA\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("kp", &kp)!=TIXML_SUCCESS) {
    printf("Could not find HAA:kp\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("kd", &kd)!=TIXML_SUCCESS) {
    printf("Could not find HAA:kd\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("maxTorque", &maxTorque)!=TIXML_SUCCESS) {
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
  if (pElem->QueryDoubleAttribute("kp", &kp)!=TIXML_SUCCESS) {
    printf("Could not find HFE:kp\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("kd", &kd)!=TIXML_SUCCESS) {
    printf("Could not find HFE:kd\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("maxTorque", &maxTorque)!=TIXML_SUCCESS) {
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

  if (pElem->QueryDoubleAttribute("kp", &kp)!=TIXML_SUCCESS) {
    printf("Could not find KFE:kp\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("kd", &kd)!=TIXML_SUCCESS) {
    printf("Could not find KFE:kd\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("maxTorque", &maxTorque)!=TIXML_SUCCESS) {
    printf("Could not find KFE:maxTorque\n");
    return false;
  }
  setJointControlGainsKFE(kp, kd);
  setMaxTorqueKFE(maxTorque);

  return true;
}

} /* namespace loco */


