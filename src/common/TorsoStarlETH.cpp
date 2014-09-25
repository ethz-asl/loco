/*
 * TorsoStarlETH.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/common/TorsoStarlETH.hpp"

namespace loco {

TorsoStarlETH::TorsoStarlETH(robotModel::RobotModel* robotModel)
    : TorsoBase(),
      robotModel_(robotModel),
      properties_(robotModel),
      stridePhase_(0.0)
{

}

TorsoStarlETH::~TorsoStarlETH()
{

}

double TorsoStarlETH::getStridePhase()
{
  return stridePhase_;
}

void TorsoStarlETH::setStridePhase(double stridePhase)
{
  stridePhase_ = stridePhase;
}

TorsoStateMeasured& TorsoStarlETH::getMeasuredState()
{
  return stateMeasured_;
}

TorsoStateDesired& TorsoStarlETH::getDesiredState()
{
  return stateDesired_;
}

const TorsoStateDesired& TorsoStarlETH::getDesiredState() const {
  return stateDesired_;
}

TorsoPropertiesBase& TorsoStarlETH::getProperties()
{
  return static_cast<TorsoPropertiesBase&>(properties_);
}

void TorsoStarlETH::setDesiredBaseTwistInHeadingFrame(const Twist& desiredBaseTwistInHeadingFrame) {
  desiredBaseTwistInHeadingFrame_ = desiredBaseTwistInHeadingFrame;
}

bool TorsoStarlETH::initialize(double dt)
{
  if(!this->getProperties().initialize(dt)) {
    return false;
  }
  if (!this->advance(dt)) {
    return false;
  }
  return true;
}

bool TorsoStarlETH::advance(double dt)
{
  if(!getProperties().advance(dt)) {
    return false;
  }

  kindr::rotations::eigen_impl::RotationQuaternionAD rquatWorldToBaseActive(
      robotModel_->est().getActualEstimator()->getQuat());
  const RotationQuaternion  orientationWorldToBase = rquatWorldToBaseActive.getPassive();
  const Position positionWorldToBaseInWorldFrame = Position(
      robotModel_->kin()[robotModel::JT_World2Base_CSw]->getPos());
//  std::cout << "world2base position: " << robotModel_->kin()[robotModel::JT_World2Base_CSw]->getPos() << std::endl;
//  std::cout << "position: " << rquatWorldToBase.rotate(robotModel_->kin()[robotModel::JT_World2Base_CSw]->getPos()) << std::endl;

  this->getMeasuredState().setPositionWorldToBaseInWorldFrame(positionWorldToBaseInWorldFrame);
  this->getMeasuredState().setOrientationWorldToBase(orientationWorldToBase);
  const LinearVelocity linearVelocityInBaseFrame(
      orientationWorldToBase.rotate(robotModel_->kin()[robotModel::JT_World2Base_CSw]->getVel()));
  const LocalAngularVelocity localAngularVelocityInBaseFrame(
      orientationWorldToBase.rotate(robotModel_->kin()(robotModel::JR_World2Base_CSw)->getOmega()));

  this->getMeasuredState().setLinearVelocityBaseInBaseFrame(linearVelocityInBaseFrame);
  this->getMeasuredState().setAngularVelocityBaseInBaseFrame(localAngularVelocityInBaseFrame);

//  std::cout << "lin vel in base frame: " << linearVelocityInBaseFrame << std::endl;


//  EulerAnglesZyx orientationWorldToHeadingEulerZyx = EulerAnglesZyx(orientationWorldToBase).getUnique();
//  orientationWorldToHeadingEulerZyx.setPitch(0.0);
//  orientationWorldToHeadingEulerZyx.setRoll(0.0);
//  RotationQuaternion orientationWorldToHeading = RotationQuaternion(orientationWorldToHeadingEulerZyx.getUnique());
//  this->getMeasuredState().setWorldToControlOrientation(orientationWorldToHeading);
//  const RotationQuaternion orientationHeadingToBase = orientationWorldToBase*orientationWorldToHeading.inverted();
//  this->getMeasuredState().setControlToBaseOrientation(orientationHeadingToBase);

  // todo fix this in torso control
//  const Twist desiredBaseTwistInBaseFrame = Twist(orientationHeadingToBase.rotate(desiredBaseTwistInHeadingFrame_.getTranslationalVelocity()),orientationHeadingToBase.rotate(desiredBaseTwistInHeadingFrame_.getRotationalVelocity()));
//  this->getDesiredState().setBaseTwistInBaseFrame(desiredBaseTwistInBaseFrame);
  return true;
}


std::ostream& operator << (std::ostream& out, const TorsoStarlETH& torso) {
//  out << "Orientation base to world (z-y-x): " << EulerAnglesZyx(torso.getDesiredState().getWorldToBaseOrientationInWorldFrame()).getUnique() << std::endl;
//  out << "Desired speed: " << torso.getDesiredState().getBaseTwistInBaseFrame();
  return out;
}




} /* namespace loco */
