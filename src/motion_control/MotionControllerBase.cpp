/*
 * MotionControllerBase.cpp
 *
 *  Created on: March 6, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/motion_control/MotionControllerBase.hpp"

using namespace std;

namespace loco {

MotionControllerBase::MotionControllerBase(std::shared_ptr<LegGroup> legs, std::shared_ptr<TorsoBase> torso)
    : legs_(legs),
      torso_(torso)
{
  isParametersLoaded_ = false;
  isLogging_ = false;
}

MotionControllerBase::~MotionControllerBase()
{

}

bool MotionControllerBase::loadParameters(const TiXmlHandle& handle)
{
  isParametersLoaded_ = true;
  return true;
}

bool MotionControllerBase::addToLogger()
{
  isLogging_ = true;
  return true;
}

bool MotionControllerBase::checkIfParametersLoaded() const
{
  if (isParametersLoaded_) return true;
  cout << "Motion controller parameters are not loaded." << endl; // TODO use warning output
  return false;
}

} /* namespace loco */
