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

LimbCoordinatorJump::LimbCoordinatorJump(LegGroup* legs, TorsoBase* torso) :
    LimbCoordinatorBase(),
    legs_(legs),
    torso_(torso),
    isLegGrounded_{false, false, false, false}
{


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


void LimbCoordinatorJump::setIsLegGrounded(int iLeg, bool isLegGrounded)
{
  isLegGrounded_[iLeg] = isLegGrounded;
}

void LimbCoordinatorJump::setShouldBeLegGrounded(int iLeg, bool shouldBeLegGrounded)
{
  shouldBeLegGrounded_[iLeg] = shouldBeLegGrounded;
}



bool LimbCoordinatorJump::initialize(double dt) {
  if(!gaitPattern_->initialize(dt)) {
    return false;
  }
  return true;
}

void LimbCoordinatorJump::advance(double dt) {
  gaitPattern_->advance(dt);
//  for (int iLeg=0; iLeg<4; iLeg++) {
  int iLeg =0;
  for (auto leg : *legs_) {
    shouldBeLegGrounded_[iLeg] = gaitPattern_->shouldBeLegGrounded(iLeg);
    leg->setShouldBeGrounded(shouldBeLegGrounded_[iLeg]);
    leg->setStanceDuration(gaitPattern_->getStanceDuration(iLeg));
    leg->setSwingDuration(gaitPattern_->getStrideDuration()-gaitPattern_->getStanceDuration(iLeg));
    leg->setWasInStanceMode(leg->isInStanceMode());
    leg->setIsInStanceMode(isLegInStanceMode(iLeg));
    leg->setWasInSwingMode(leg->isInSwingMode());
    leg->setIsInSwingMode(isLegInSwingMode(iLeg));
    leg->setSwingPhase(gaitPattern_->getSwingPhaseForLeg(iLeg));
    leg->setStancePhase(gaitPattern_->getStancePhaseForLeg(iLeg));
    iLeg++;
  }

  torso_->setStridePhase(gaitPattern_->getStridePhase());
}

bool LimbCoordinatorJump::isLegInStanceMode(int iLeg) {
  const double swingPhase = gaitPattern_->getSwingPhaseForLeg(iLeg);
  if (swingPhase > 0.9 && this->isLegGrounded(iLeg)) return true; // early touch-down -> switch to stance mode
  return (swingPhase < 0 || swingPhase > 1); // stance mode
}


bool LimbCoordinatorJump::isLegInSwingMode(int iLeg) {
  const double swingPhase = gaitPattern_->getSwingPhaseForLeg(iLeg);
  if (swingPhase > 0.9 && this->isLegGrounded(iLeg)) return false; // early touch-down -> switch to stance mode
  return (swingPhase >= 0 && swingPhase <= 1);
}

GaitPatternBase* LimbCoordinatorJump::getGaitPattern()
{
  return gaitPattern_;
}

bool LimbCoordinatorJump::loadParameters(const TiXmlHandle& handle)
{
  if (!gaitPattern_->loadParameters(TiXmlHandle(handle.FirstChild("LimbCoordination")))) {
    return false;
  }
  return true;
}

} /* namespace loco */
