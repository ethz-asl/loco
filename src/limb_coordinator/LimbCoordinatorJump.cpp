/*!
 * @file     LimbCoordinatorJump.cpp
 * @author   wko
 * @date     Jun, 2014
 * @version  1.0
 * @ingroup
 * @brief
 */
#include "loco/limb_coordinator/LimbCoordinatorJump.hpp"

namespace loco {

LimbCoordinatorJump::LimbCoordinatorJump(LegGroup* legs, TorsoBase* torso)
    : LimbCoordinatorBase(),
      legs_(legs),
      torso_(torso),
      isLegGrounded_ { false, false, false, false } {

}

LimbCoordinatorJump::~LimbCoordinatorJump() {

}

bool LimbCoordinatorJump::isLegGrounded(int iLeg) {
  return isLegGrounded_[iLeg];
}

bool LimbCoordinatorJump::shouldBeLegGrounded(int iLeg) {
  return shouldBeLegGrounded_[iLeg];
}
bool LimbCoordinatorJump::isAndShouldBeLegGrounded(int iLeg) {
  return (isLegGrounded_[iLeg] && shouldBeLegGrounded_[iLeg]);
}

void LimbCoordinatorJump::setIsLegGrounded(int iLeg, bool isLegGrounded) {
  isLegGrounded_[iLeg] = isLegGrounded;
}

void LimbCoordinatorJump::setShouldBeLegGrounded(int iLeg,
                                                 bool shouldBeLegGrounded) {
  shouldBeLegGrounded_[iLeg] = shouldBeLegGrounded;
}

bool LimbCoordinatorJump::initialize(double dt) {
  int iLeg = 0;

  // Legs are just on the ground prior to jump
  for (auto leg : *legs_) {
    shouldBeLegGrounded_[iLeg] = true;
    leg->setShouldBeGrounded(shouldBeLegGrounded_[iLeg]);
    leg->setIsInStanceMode(true);
    leg->setWasInStanceMode(true);
    leg->setIsInSwingMode(false);
    leg->setWasInSwingMode(false);
    leg->setIsGrounded(true);

    iLeg++;
  }


  return true;
}

void LimbCoordinatorJump::advance(double dt) {
  int iLeg = 0;

  // Set legs in stance or swing mode by checking ground contact
  for (auto leg : *legs_) {
    leg->setIsInStanceMode(leg->isGrounded());
    leg->setIsInSwingMode(!leg->isGrounded());
    iLeg++;
  }
}

bool LimbCoordinatorJump::isLegInStanceMode(int iLeg) {
  return true;
}

bool LimbCoordinatorJump::isLegInSwingMode(int iLeg) {
  return true;
}

// Not needed here, but implemented since it's virtual...
GaitPatternBase* LimbCoordinatorJump::getGaitPattern() {
  return NULL;
}

bool LimbCoordinatorJump::loadParameters(const TiXmlHandle& handle) {
  return true;
}

} /* namespace loco */
