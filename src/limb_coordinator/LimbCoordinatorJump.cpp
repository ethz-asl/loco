/*
 * LimbCoordinatorJump.cpp
 *
 *  Created on: Oct 6, 2014
 *      Author: gech
 */

#include "LimbCoordinatorJump.hpp"

namespace loco {

LimbCoordinatorJump::LimbCoordinatorJump(LegGroup* legs, TorsoBase* torso) : LimbCoordinatorBase(),
    legs_(legs),
    torso_(torso)
{

}

LimbCoordinatorJump::~LimbCoordinatorJump()
{
}

bool LimbCoordinatorJump::initialize(double dt) {
  return true;
}

bool LimbCoordinatorJump::advance(double dt) {

  StateSwitcher* stateSwitcher;

   for (auto leg : *legs_) {

     stateSwitcher = leg->getStateSwitcher();
     stateSwitcher->setState(StateSwitcher::States::StanceNormal);
     leg->setIsSupportLeg(true);

   }
  return true;
}

} /* namespace loco */
