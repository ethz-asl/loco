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

bool MotionControllerBase::checkIfParametersLoaded() const
{
  if (isParametersLoaded_) return true;
  cout << "Motion controller parameters are not loaded." << endl; // TODO use warning output
  return false;
}

bool MotionControllerBase::setToInterpolated(const MotionControllerBase& motionController1, const MotionControllerBase& motionController2, double t) {
  return true;
}

} /* namespace loco */
