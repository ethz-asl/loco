/*
 * ContactForceDistribution.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "ContactForceDistribution.hpp"

using namespace std;
using namespace Eigen;

namespace robotController {

ContactForceDistribution::ContactForceDistribution(robotModel::RobotModel* robotModel)
{
  robotModel_ = robotModel;
  legLoadFactors_.setOnes();
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

  cout << "Contact force distribution parameters are not loaded." << endl; // TODO use warning output
  return false;
}

bool ContactForceDistribution::changeLegLoad(const LegName& legName,
                                             const double& loadFactor)
{
  legLoadFactors_(legName) = loadFactor; // Dangerous!
}

bool ContactForceDistribution::computeForceDistribution(
    const Eigen::Vector3d& desiredVirtualForce,
    const Eigen::Vector3d& desiredVirtualTorque)
{
  prepareDesiredLegLoading();
  prepareOptimization();
}

bool ContactForceDistribution::prepareDesiredLegLoading()
{
  legLoadFactors_ = legLoadFactors_.array() * robotModel_->contacts().getCA().cast<double>().array();
}

bool ContactForceDistribution::prepareOptimization()
{
}

} /* namespace robotController */
