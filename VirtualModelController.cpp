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
}

VirtualModelController::~VirtualModelController()
{
  delete contactForceDistribution_;
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

  computePoseError();
  computeVirtualForce();
  computeContactForces();

  return true;
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
  return true;
}

bool VirtualModelController::computeContactForces()
{
//  contactForceDistribution_->run();
  return true;
}

} /* namespace robotController */
