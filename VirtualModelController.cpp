/*
 * VirtualModelController.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "VirtualModelController.hpp"

// robotUtils
//#include "Logger.hpp"

using namespace std;
using namespace robotModel;
using namespace robotController;
using namespace robotUtils;
using namespace Eigen;

namespace robotController {

VirtualModelController::VirtualModelController(RobotModel* robotModel) : ControllerBase(robotModel)
{
  contactForceDistribution_ = new ContactForceDistribution();
  isParametersLoaded = false;
}

VirtualModelController::~VirtualModelController()
{
  delete contactForceDistribution_;
}

bool VirtualModelController::loadParameters()
{
  // TODO Replace this with proper parameters loading (XML)
  proportionalGainTranslation_ << 500, 640, 600;
  derivativeGainTranslation_ << 150, 100, 120;
  feedforwardGainTranslation_ << 25, 0, 1;
  proportionalGainRotation_ << 400, 200, 0;
  derivativeGainRotation_ << 6, 9, 0;
  feedforwardGainRotation_ << 0, 0, 0;

  isParametersLoaded = true;

  return true;
}

bool VirtualModelController::computeTorques(robotModel::VectorP baseDesiredPosition,
                                            Eigen::Quaterniond baseDesiredOrientation,
                                            robotModel::VectorP baseDesiredLinearVelocity,
                                            robotModel::VectorO baseDesiredAngularVelocity)
{
  desiredPosition_ = baseDesiredPosition;
  desiredOrientation_ = baseDesiredOrientation;
  desiredVelocity_ = baseDesiredLinearVelocity;
  desiredAngularVelocity_ = baseDesiredAngularVelocity;

  if (!isParametersLoaded)
  {
    cout << "Virtual model control parameters are not loaded." << endl; // TODO use warning output
    return false;
  }
  else
  {
    computePoseError();
    computeVirtualForce();
    computeVirtualTorque();
    computeContactForces();
    return true;
  }
}

bool VirtualModelController::computePoseError()
{
  positionError_ = desiredPosition_ - robotModel_->kin().getJacobianT(JT_World2Base_CSw)->getPos();
  velocityError_ = desiredVelocity_ - robotModel_->kin().getJacobianT(JT_World2Base_CSw)->getVel();
  angularVelocityError_ = desiredAngularVelocity_ - robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getOmega();

  Quaterniond actualOrientation = Quaterniond(robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA());
  orientationError_ = desiredOrientation_.conjugate() * actualOrientation;

  return true;
}

bool VirtualModelController::computeVirtualForce()
{
  Vector3d feedforwardTerm = Vector3d::Zero();
  feedforwardTerm.x() = desJointVelocities_.x();
  feedforwardTerm.y() = desJointVelocities_.y();
  feedforwardTerm.z() = robotModel_->params().mTotal_ * robotModel_->params().gravity_;

  desiredVirtualForce_ = proportionalGainTranslation_.array() * positionError_.array() // Coefficient-wise multiplication.
                       + derivativeGainTranslation_.array()   * velocityError_.array()
                       + feedforwardGainTranslation_.array()  * feedforwardTerm.array();

  desiredVirtualForce_ = robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA() * desiredVirtualForce_;

  return true;
}

bool VirtualModelController::computeVirtualTorque()
{
  return true;
}

bool VirtualModelController::computeContactForces()
{
//  contactForceDistribution_->run();
  return true;
}

} /* namespace robotController */
