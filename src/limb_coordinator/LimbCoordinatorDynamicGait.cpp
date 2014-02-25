/*!
* @file     LimbCoordinatorDynamicGait.cpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#include "loco/limb_coordinator/LimbCoordinatorDynamicGait.hpp"

namespace loco {

LimbCoordinatorDynamicGait::LimbCoordinatorDynamicGait(GaitPatternBase* gaitPattern) :
    LimbCoordinatorBase(),
    gaitPattern_(gaitPattern),
    isLegGrounded_{false, false, false, false}
{


}

LimbCoordinatorDynamicGait::~LimbCoordinatorDynamicGait() {

}

bool LimbCoordinatorDynamicGait::isLegGrounded(int iLeg) {
    return isLegGrounded_[iLeg];
}

bool LimbCoordinatorDynamicGait::shouldBeLegGrounded(int iLeg) {
  return shouldBeLegGrounded_[iLeg];
}
bool LimbCoordinatorDynamicGait::isAndShouldBeLegGrounded(int iLeg) {
  return (isLegGrounded_[iLeg] && shouldBeLegGrounded_[iLeg]);
}


void LimbCoordinatorDynamicGait::setIsLegGrounded(int iLeg, bool isLegGrounded)
{
  isLegGrounded_[iLeg] = isLegGrounded;
}

void LimbCoordinatorDynamicGait::setShouldBeLegGrounded(int iLeg, bool shouldBeLegGrounded)
{
  shouldBeLegGrounded_[iLeg] = shouldBeLegGrounded;
}


void LimbCoordinatorDynamicGait::advance(LegGroup& legs, double dt) {
  gaitPattern_->advance(dt);
//  for (int iLeg=0; iLeg<4; iLeg++) {
  int iLeg =0;
  for (auto leg : legs) {
    shouldBeLegGrounded_[iLeg] = gaitPattern_->shouldBeLegGrounded(iLeg);
    leg->setShouldBeGrounded(shouldBeLegGrounded_[iLeg]);
    leg->setStanceDuration(gaitPattern_->getStanceDuration(iLeg));
    leg->setSwingDuration(gaitPattern_->getStrideDuration()-gaitPattern_->getStanceDuration(iLeg));
    leg->setIsInStanceMode(isLegInStanceMode(iLeg));
    leg->setIsInSwingMode(isLegInSwingMode(iLeg));
    iLeg++;
  }
}

bool LimbCoordinatorDynamicGait::isLegInStanceMode(int iLeg) {
  const double swingPhase = gaitPattern_->getSwingPhaseForLeg(iLeg);
  if (swingPhase > 0.9 && this->isLegGrounded(iLeg)) return true;
  return (swingPhase < 0 || swingPhase > 1);
}


bool LimbCoordinatorDynamicGait::isLegInSwingMode(int iLeg) {
  const double swingPhase = gaitPattern_->getSwingPhaseForLeg(iLeg);
  if (swingPhase > 0.9 && this->isLegGrounded(iLeg)) return false;
  return (swingPhase >= 0 && swingPhase <= 1);
}

GaitPatternBase* LimbCoordinatorDynamicGait::getGaitPattern()
{
  return gaitPattern_;
}

} /* namespace loco */
