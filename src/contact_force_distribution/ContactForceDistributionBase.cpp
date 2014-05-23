/*
 * ContactForceDistributionBase.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/contact_force_distribution/ContactForceDistributionBase.hpp"

using namespace std;
using namespace Eigen;

namespace loco {

ContactForceDistributionBase::ContactForceDistributionBase(std::shared_ptr<TorsoBase> torso, std::shared_ptr<LegGroup> legs, std::shared_ptr<loco::TerrainModelBase> terrain)
: torso_(torso),
  legs_(legs),
  terrain_(terrain)
{
  isParametersLoaded_ = false;
  isLogging_ = false;
  isForceDistributionComputed_ = false;
}

ContactForceDistributionBase::~ContactForceDistributionBase()
{

}

bool ContactForceDistributionBase::checkIfParametersLoaded() const
{
  if (isParametersLoaded_) return true;
  cout << "Contact force distribution parameters are not loaded." << endl; // TODO use warning output
  return false;
}

bool ContactForceDistributionBase::checkIfForceDistributionComputed() const
{
  if (isForceDistributionComputed_) return true;
//  cout << "Contact force distribution is not computed yet or was unsuccessful." << endl; // TODO use warning output
  return false;
}

} /* namespace loco */
