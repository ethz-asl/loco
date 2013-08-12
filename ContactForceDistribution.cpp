/*
 * ContactForceDistribution.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "ContactForceDistribution.hpp"
#include "NumericalComparison.hpp"

using namespace std;
using namespace Eigen;
using namespace robotUtils;

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
    const Eigen::Vector3d& virtualForce,
    const Eigen::Vector3d& virtualTorque)
{
  prepareDesiredLegLoading();
  prepareOptimization(virtualForce, virtualTorque);

  legLoadFactors_.setOnes();
}

bool ContactForceDistribution::prepareDesiredLegLoading()
{
  legLoadFactors_ = legLoadFactors_.array() * robotModel_->contacts().getCA().cast<double>().array();

  nLegsInStance_ = 0;
  for(int i = 0; i < legLoadFactors_.rows(); i++)
  {
    if(NumericalComparison::definitelyGreaterThan(legLoadFactors_(i), 0.0))
      nLegsInStance_++;
  }
}

bool ContactForceDistribution::prepareOptimization(
    const Eigen::Vector3d& virtualForce,
    const Eigen::Vector3d& virtualTorque)
{
  stackedVirtualForceAndTorque_.setZero();
  stackedVirtualForceAndTorque_.segment(0, 3) = virtualForce;
  stackedVirtualForceAndTorque_.segment(3, 3) = virtualTorque;

  virtualForceWeightsMatrix_.setZero();
  virtualForceWeightsMatrix_ = virtualForceWeights_.asDiagonal();

  stackedContactForces_.Zero(N_TRANSLATIONAL_DOF_PER_FOOT * nLegsInStance_);

  optimizationMatrixA_.resize(6, N_TRANSLATIONAL_DOF_PER_FOOT * nLegsInStance_);
  optimizationMatrixA_.setZero();
  optimizationMatrixA_.topRows(3) = Matrix3d::Identity().replicate(1, nLegsInStance_);
  // TODO finish

  groundForceWeights_.setIdentity(N_TRANSLATIONAL_DOF_PER_FOOT * nLegsInStance_);
  groundForceWeights_ = groundForceWeights_ * groundForceWeight_;
}

} /* namespace robotController */
