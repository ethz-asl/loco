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
using namespace robotUtils;
using namespace robotModel;

namespace loco {

ContactForceDistributionBase::ContactForceDistributionBase(robotModel::RobotModel* robotModel)
{
  robotModel_ = robotModel;
  terrain_ = nullptr;
  isParametersLoaded_ = false;
  isTerrainSet_ = false;
  isForceDistributionComputed_ = false;
}

ContactForceDistributionBase::~ContactForceDistributionBase()
{

}

bool ContactForceDistributionBase::loadParameters()
{
  isParametersLoaded_ = true;
  return true;
}

bool ContactForceDistributionBase::setTerrain(const robotTerrain::TerrainBase* terrain)
{
  if (terrain != nullptr)
  {
    terrain_ = terrain;
    isTerrainSet_ = true;
  }
  return false;
}

bool ContactForceDistributionBase::addToLogger()
{
  return true;
}

bool ContactForceDistributionBase::isParametersLoaded() const
{
  if (isParametersLoaded_) return true;
  cout << "Contact force distribution parameters are not loaded." << endl; // TODO use warning output
  return false;
}

bool ContactForceDistributionBase::isForceDistributionComputed() const
{
  if (isForceDistributionComputed_) return true;
  cout << "Contact force distribution is not computed yet or was unsuccessful." << endl; // TODO use warning output
  return false;
}

} /* namespace loco */
