/*
 * ContactForceDistribution.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "ContactForceDistribution.hpp"
#include <Eigen/SparseCore>
#include "LinearAlgebra.hpp"
#include "NumericalComparison.hpp"
#include "OoqpInterface.hpp"
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
  for (LegNumber legNumber = LEFT_FRONT; legNumber <= RIGHT_HIND; legNumber = LegNumber(legNumber+1))
  {
    legStatuses_[legNumber] = LegStatus();
  }

  resetFootLoadFactors();
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

bool ContactForceDistribution::changeLegLoad(const LegNumber& legNumber,
                                             const double& loadFactor)
{
  //legLoadFactors_(legName) = loadFactor; // Dangerous!
}

bool ContactForceDistribution::computeForceDistribution(
    const Eigen::Vector3d& virtualForce,
    const Eigen::Vector3d& virtualTorque)
{
  prepareDesiredLegLoading();
  if (nLegsInStance_ > 0)
  {
    prepareOptimization(virtualForce, virtualTorque);
    solveOptimization();
  }
  resetFootLoadFactors();
}

bool ContactForceDistribution::prepareDesiredLegLoading()
{
  nLegsInStance_ = 0;

  for (auto& legStatus : legStatuses_)
  {
    legStatus.second.loadFactor_ = legStatus.second.loadFactor_
        * robotModel_->contacts().getCA().cast<double>()(legStatus.first);

    if (NumericalComparison::definitelyGreaterThan(legStatus.second.loadFactor_, 0.0))
    {
      legStatus.second.isInStance_ = true;
      legStatus.second.startIndexInVectorX_ = nLegsInStance_ * nTranslationalDofPerFoot_;
      nLegsInStance_++;
    }
    else
    {
      legStatus.second.isInStance_ = false;
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
  A_.middleRows(0, nTranslationalDofPerFoot_) = (Matrix3d::Identity().replicate(1, nLegsInStance_)).sparseView();

  MatrixXd A_bottomMatrix(nTranslationalDofPerFoot_, n_);
  int i = 0;
  for (auto& legStatus : legStatuses_)
  {
    if (legStatus.second.isInStance_)
    {
      // TODO coordinate system?
      VectorP r = robotModel_->kin().getJacobianTByLeg_Base2Foot_CSw(legStatus.first)->getPos();
      A_bottomMatrix.block(0, i * r.size(), r.size(), r.size()) = getSkewMatrixFromVector(r);
      i++;
    }
  }
  A_.middleRows(nTranslationalDofPerFoot_, A_bottomMatrix.rows()) = A_bottomMatrix.sparseView();

  W_.setIdentity(nTranslationalDofPerFoot_ * nLegsInStance_);
  W_ = W_ * groundForceWeight_;
}

bool ContactForceDistribution::solveOptimization()
{
  Eigen::SparseMatrix<double, Eigen::RowMajor> C;
  Eigen::VectorXd c;
  return QuadraticProblemFormulation::solve(A_, S_, b_, W_, C, c, D_, d_, f_, x_);
}

bool robotController::ContactForceDistribution::resetFootLoadFactors()
{
  for (auto& legStatus : legStatuses_)
  {
    legStatus.second.loadFactor_ = 1.0;
  }

  return true;
}

bool ContactForceDistribution::getForceForLeg(
    const LegNumber& legNumber, robotModel::VectorCF& force)
{
  force = VectorCF();

  if (legStatuses_[legNumber].isInStance_)
  {
    // The forces we computed here are actually the ground reaction forces,
    // so the stance legs should push the ground by the opposite amount.
    force = -x_.segment(legStatuses_[legNumber].startIndexInVectorX_, nTranslationalDofPerFoot_);
    return true;
  }

  return false;
}

} /* namespace robotController */
