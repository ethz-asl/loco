/*
 * ContactDetectorConstantDuringStance.cpp
 *
 *  Created on: May 26, 2014
 *      Author: gech
 */

#include "loco/contact_detection/ContactDetectorConstantDuringStance.hpp"

namespace loco {

ContactDetectorConstantDuringStance::ContactDetectorConstantDuringStance(LegGroup* legs) :
    ContactDetectorBase(),
    legs_(legs)
{


}

ContactDetectorConstantDuringStance::~ContactDetectorConstantDuringStance() {

}

bool ContactDetectorConstantDuringStance::initialize(double dt) {
  for (auto leg :  *legs_) {
    registeredContact_[leg->getId()] = leg->isGrounded();
  }

  return true;
}

bool ContactDetectorConstantDuringStance::advance(double dt) {

  for (auto leg :  *legs_) {
    int i = leg->getId();
    if (leg->isInSwingMode())
      registeredContact_[i] = false;
    if (leg->isInStanceMode() && leg->isGrounded())
      registeredContact_[i] = true;


    leg->setIsGrounded( registeredContact_[i] || leg->isGrounded());
  }
  return true;
}



} /* namespace loco */
