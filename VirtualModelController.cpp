/*
 * VirtualModelController.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "VirtualModelController.hpp"
#include "Logger.hpp"

using namespace std;
using namespace robotModel;
using namespace robotController;
using namespace robotUtils;
using namespace Eigen;

namespace robotController {

VirtualModelController::VirtualModelController(RobotModel* robotModel) : ControllerBase(robotModel)
{
  contactForceDistribution_ = new ContactForceDistribution(robotModel);
  isParametersLoaded_ = false;
  virtualForce_ = Vector3d::Zero();
  virtualTorque_ = Vector3d::Zero();
}

VirtualModelController::~VirtualModelController()
{
  delete contactForceDistribution_;
}

bool VirtualModelController::initialize()
{
  addEigenMatrixToLog(virtualForce_, "VMC_desired_force", "N", true);
  addEigenMatrixToLog(virtualTorque_, "VMC_desired_torque", "Nm", true);
  updateLogger();
  return true;
}

bool VirtualModelController::loadParameters()
{
  // TODO Replace this with proper parameters loading (XML)
  proportionalGainTranslation_ << 500.0, 640.0, 600.0;
  derivativeGainTranslation_ << 150.0, 100.0, 120.0;
  feedforwardGainTranslation_ << 25.0, 0.0, 1.0;
  proportionalGainRotation_ << 400.0, 200.0, 0.0;
  derivativeGainRotation_ << 6.0, 9.0, 0.0;
  feedforwardGainRotation_ << 0.0, 0.0, 0.0;

  contactForceDistribution_->loadParameters();

  isParametersLoaded_ = true;

  return true;
}

bool VirtualModelController::computeTorques(
    const robotModel::VectorP& desiredPosition,
    const Eigen::Quaterniond& desiredOrientation,
    const robotModel::VectorP& desiredLinearVelocity,
    const robotModel::VectorO& desiredAngularVelocity)
{
  if (!areParametersLoaded()) return false;

  computePoseError(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
  computeVirtualForce(desiredLinearVelocity);
  computeVirtualTorque(desiredAngularVelocity);
  computeContactForces();
  return true;
}

bool VirtualModelController::computePoseError(
    const robotModel::VectorP& desiredPosition,
    const Eigen::Quaterniond& desiredOrientation,
    const robotModel::VectorP& desiredLinearVelocity,
    const robotModel::VectorO& desiredAngularVelocity)
{
  MatrixA transformationFromWorldToBase = robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA();

  positionError_ = desiredPosition - robotModel_->kin().getJacobianT(JT_World2Base_CSw)->getPos();
  positionError_ = transformationFromWorldToBase * positionError_;

  Quaterniond actualOrientation = Quaterniond(robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA());
  Quaterniond orientationError = desiredOrientation.conjugate() * actualOrientation;
  orientationErrorVector_ = AngleAxisd(orientationError).angle() * AngleAxisd(orientationError).axis();
  orientationErrorVector_ = transformationFromWorldToBase * orientationErrorVector_;

  linearVelocityError_ = desiredLinearVelocity - robotModel_->kin().getJacobianT(JT_World2Base_CSw)->getVel();
  linearVelocityError_ = transformationFromWorldToBase * linearVelocityError_;

  angularVelocityError_ = desiredAngularVelocity - robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getOmega();
  angularVelocityError_ = transformationFromWorldToBase * angularVelocityError_;

  return true;
}

bool VirtualModelController::computeVirtualForce(const robotModel::VectorP& desiredLinearVelocity)
{
  Vector3d feedforwardTerm = Vector3d::Zero();
  feedforwardTerm.x() = desiredLinearVelocity.x();
  feedforwardTerm.y() = desiredLinearVelocity.y();
  feedforwardTerm.z() = robotModel_->params().mTotal_ * robotModel_->params().gravity_;

  virtualForce_ = proportionalGainTranslation_.array() * positionError_.array() // Coefficient-wise multiplication.
                       + derivativeGainTranslation_.array()   * linearVelocityError_.array()
                       + feedforwardGainTranslation_.array()  * feedforwardTerm.array();

  // TODO figure this out which frame
  //desiredVirtualForce_ = robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA() * desiredVirtualForce_;

  return true;
}

bool VirtualModelController::computeVirtualTorque(const robotModel::VectorO& desiredAngularVelocity)
{
  Vector3d feedforwardTerm = Vector3d::Zero();
  feedforwardTerm.z() = desiredAngularVelocity.z();

  virtualTorque_ = proportionalGainRotation_.array() * orientationErrorVector_.array()
                       + derivativeGainRotation_.array()   * angularVelocityError_.array()
                       + feedforwardGainRotation_.array()  * feedforwardTerm.array();

  // TODO figure this out which frame
  //desiredVirtualTorque_ = robotModel_->kin().getJacobianR(JR_World2Base_CSw)->getA() * desiredVirtualTorque_;

  return true;
}

bool VirtualModelController::computeContactForces()
{
//  contactForceDistribution_->run();
  return true;
}

bool VirtualModelController::areParametersLoaded()
{
  if (isParametersLoaded_) return true;

  cout << "Virtual model control parameters are not loaded." << endl; // TODO use warning output
  return false;
}

} /* namespace robotController */
