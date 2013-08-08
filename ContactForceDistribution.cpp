/*
 * ContactForceDistribution.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "ContactForceDistribution.hpp"

using namespace std;

namespace robotController {


ContactForceDistribution::ContactForceDistribution(
    robotModel::RobotModel* robotModel)
{

}

ContactForceDistribution::~ContactForceDistribution()
{
}

bool ContactForceDistribution::loadParameters()
{
  // TODO Replace this with proper parameters loading (XML)
  virtualForceWeights_ << 1.0, 1.0, 1.0, 10.0, 10.0, 5.0;
  groundForceWeight_ = 0.00001;
  minimalNormalGroundForce_  = 2.0;
  frictionCoefficient_ = 0.8;

  isParametersLoaded_ = true;

  return true;
}

bool ContactForceDistribution::areParametersLoaded()
{
  if (isParametersLoaded_) return true;

  cout << "Virtual model control parameters are not loaded." << endl; // TODO use warning output
  return false;
}

bool ContactForceDistribution::computeForceDistribution(
    const Eigen::Vector3d& desiredVirtualForce,
    const Eigen::Vector3d& desiredVirtualTorque)
{
  prepareOptimization();
}

bool ContactForceDistribution::prepareOptimization()
{
}

} /* namespace robotController */
