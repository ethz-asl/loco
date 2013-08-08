/*
 * VirtualModelController.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "VirtualModelController.hpp"

// robotUtils
#include "Logger.hpp"

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

  desiredVirtualForce_ = Vector3d::Zero();
  desiredVirtualTorque_ = Vector3d::Zero();
}

VirtualModelController::~VirtualModelController()
{
  delete contactForceDistribution_;
}

bool VirtualModelController::initialize()
{
  addEigenMatrixToLog(desiredVirtualForce_, "VMC_desired_force", "N", true);
  addEigenMatrixToLog(desiredVirtualTorque_, "VMC_desired_torque", "Nm", true);

  updateLogger();
  return true;
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

bool VirtualModelController::computeTorques(robotModel::VectorP desiredPosition,
                                            Eigen::Quaterniond desiredOrientation,
                                            robotModel::VectorP desiredLinearVelocity,
                                            robotModel::VectorO desiredAngularVelocity)
{
  if (!checkIfParametersLoaded()) return false;

  computePoseError(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
  computeVirtualForce(desiredLinearVelocity);
  computeVirtualTorque(desiredAngularVelocity);
  computeContactForces();
  return true;
}

bool VirtualModelController::computePoseError(robotModel::VectorP desiredPosition,
                                              Eigen::Quaterniond desiredOrientation,
                                              robotModel::VectorP desiredLinearVelocity,
                                              robotModel::VectorO desiredAngularVelocity)
{
  MatrixA transformationFromWorldToBody = robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA();

  positionError_ = desiredPosition - robotModel_->kin().getJacobianT(JT_World2Base_CSw)->getPos();
  positionError_ = transformationFromWorldToBody * positionError_;

  Quaterniond actualOrientation = Quaterniond(robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA());
  Quaterniond orientationError = desiredOrientation.conjugate() * actualOrientation;
  orientationErrorVector_ = AngleAxisd(orientationError).angle() * AngleAxisd(orientationError).axis();
  orientationErrorVector_ = transformationFromWorldToBody * orientationErrorVector_;

  linearVelocityError_ = desiredLinearVelocity - robotModel_->kin().getJacobianT(JT_World2Base_CSw)->getVel();
  linearVelocityError_ = transformationFromWorldToBody * linearVelocityError_;

  angularVelocityError_ = desiredAngularVelocity - robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getOmega();
  angularVelocityError_ = transformationFromWorldToBody * angularVelocityError_;

  return true;
}

bool VirtualModelController::computeVirtualForce(robotModel::VectorP desiredLinearVelocity)
{
  Vector3d feedforwardTerm = Vector3d::Zero();
  feedforwardTerm.x() = desiredLinearVelocity.x();
  feedforwardTerm.y() = desiredLinearVelocity.y();
  feedforwardTerm.z() = robotModel_->params().mTotal_ * robotModel_->params().gravity_;

  desiredVirtualForce_ = proportionalGainTranslation_.array() * positionError_.array() // Coefficient-wise multiplication.
                       + derivativeGainTranslation_.array()   * linearVelocityError_.array()
                       + feedforwardGainTranslation_.array()  * feedforwardTerm.array();

  desiredVirtualForce_ = robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA() * desiredVirtualForce_;

  return true;
}

bool VirtualModelController::computeVirtualTorque(robotModel::VectorO desiredAngularVelocity)
{
  Vector3d feedforwardTerm = Vector3d::Zero();
  feedforwardTerm.z() = desiredAngularVelocity.z();

  desiredVirtualTorque_ = proportionalGainRotation_.array() * orientationErrorVector_.array()
                       + derivativeGainRotation_.array()   * angularVelocityError_.array()
                       + feedforwardGainRotation_.array()  * feedforwardTerm.array();

  desiredVirtualTorque_ = robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA() * desiredVirtualTorque_;

  return true;
}

bool VirtualModelController::computeContactForces()
{
//  contactForceDistribution_->run();
  return true;
}

bool VirtualModelController::checkIfParametersLoaded()
{
  if (isParametersLoaded) return true;

  cout << "Virtual model control parameters are not loaded." << endl; // TODO use warning output
  return false;
}

} /* namespace robotController */
