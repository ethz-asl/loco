/*
 * ContactForceDistribution.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "ContactForceDistribution.hpp"
#include "NumericalComparison.hpp"
#include "OoqpInterface.hpp"
#include "QuadraticProblemFormulation.cpp"
#include <Eigen/SparseCore>
#include "QuadraticProblemFormulation.hpp"

using namespace std;
using namespace Eigen;
using namespace robotUtils;
using namespace robotModel;

namespace robotController {

ContactForceDistribution::ContactForceDistribution(robotModel::RobotModel* robotModel)
{
  robotModel_ = robotModel;
  legLoadFactors_.setOnes(nLegs_);

  // TODO dangerous!
  for (LegName legName = LEFT_FRONT; legName <= RIGHT_HIND; legName = LegName(legName+1))
  {
    legStatuses_[legName] = LegStatus();
    legStatuses_[legName].indexInContactArray_ = legName;
  }

  reset();
}

ContactForceDistribution::~ContactForceDistribution()
{
}

bool ContactForceDistribution::loadParameters()
{
  // TODO Replace this with proper parameters loading (XML)
  virtualForceWeights_.resize(nElementsInStackedVirtualForceTorqueVector_);
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
  //legLoadFactors_(legName) = loadFactor; // Dangerous!
}

bool ContactForceDistribution::computeForceDistribution(
    const Eigen::Vector3d& virtualForce,
    const Eigen::Vector3d& virtualTorque)
{
  prepareDesiredLegLoading();
  prepareOptimization(virtualForce, virtualTorque);
  solveOptimization();
  reset();
}

bool ContactForceDistribution::prepareDesiredLegLoading()
{
  nLegsInStance_ = 0;

  for (map<LegName, LegStatus>::iterator legStatus = legStatuses_.begin();
      legStatus != legStatuses_.end(); legStatus++)
  {
    legStatus->second.loadFactor_ = legStatus->second.loadFactor_
        * robotModel_->contacts().getCA().cast<double>()(legStatus->second.indexInContactArray_);

    if (NumericalComparison::definitelyGreaterThan(legStatus->second.loadFactor_, 0.0))
    {
      legStatus->second.isInStance_ = true;
      legStatus->second.startIndexInVectorX_ = nLegsInStance_ * nTranslationalDofPerFoot_;
      nLegsInStance_++;
    }
    else
    {
      legStatus->second.isInStance_ = false;
    }
  }
}

bool ContactForceDistribution::prepareOptimization(
    const Eigen::Vector3d& virtualForce,
    const Eigen::Vector3d& virtualTorque)
{
  n_ = nTranslationalDofPerFoot_ * nLegsInStance_;

  b_.resize(nElementsInStackedVirtualForceTorqueVector_);
  b_.segment(0, virtualForce.size()) = virtualForce;
  b_.segment(virtualForce.size(), virtualTorque.size()) = virtualTorque;

  S_ = virtualForceWeights_.asDiagonal();

  A_.resize(nElementsInStackedVirtualForceTorqueVector_, n_);
  A_.setZero();
  A_.middleRows(0, 3) = (Matrix3d::Identity().replicate(1, nLegsInStance_)).sparseView();
  // TODO finish
//  cout << "------A----" << endl;
//  cout << A_.toDense() << endl;
//  cout << "------B----" << endl;

  W_.setIdentity(nTranslationalDofPerFoot_ * nLegsInStance_);
  W_ = W_ * groundForceWeight_;
}

bool ContactForceDistribution::solveOptimization()
{
  Eigen::SparseMatrix<double, Eigen::RowMajor> C;
  Eigen::VectorXd c;
  return QuadraticProblemFormulation::solve(A_, S_, b_, W_, C, c, D_, d_, f_, x_);
}

bool robotController::ContactForceDistribution::reset()
{
  for (map<LegName, LegStatus>::iterator legStatus = legStatuses_.begin();
      legStatus != legStatuses_.end(); legStatus++)
  {
    legStatus->second.loadFactor_ = 1.0;
  }

  return true;
}

bool ContactForceDistribution::getForceForLeg(
    const LegName& legName, robotModel::VectorCF& force)
{
  force = VectorCF();

  if (legStatuses_[legName].isInStance_)
  {
    // The forces we computed here are actually the ground reaction forces,
    // so the stance legs should push the ground by the opposite amount.
    force = -x_.segment(legStatuses_[legName].startIndexInVectorX_, nTranslationalDofPerFoot_);
    return true;
  }

  return false;
}

} /* namespace robotController */
