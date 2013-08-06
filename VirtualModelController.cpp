/*
 * VirtualModelController.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "VirtualModelController.hpp"

using namespace std;
using namespace robotModel;
using namespace robotController;

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
               robotModel::VectorRPY baseDesiredOrientation,
               robotModel::VectorP baseDesiredLinearVelocity,
               robotModel::VectorO baseDesiredAngularVelocity)
{
  desiredPosition_ = baseDesiredPosition;
  desiredOrientation_ = baseDesiredOrientation;
  desiredLinearVelocity_ = baseDesiredLinearVelocity;
  desiredAngularVelocity_ = baseDesiredAngularVelocity;

  computePoseError();
  computeVirtualForce();
  computeContactForces();

  return true;
}

bool VirtualModelController::computePoseError()
{


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
