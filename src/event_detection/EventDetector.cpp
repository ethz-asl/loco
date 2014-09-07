/*
 * EventDetector.cpp
 *
 *  Created on: Sep 5, 2014
 *      Author: C. Dario Bellicoso
 */


#include "loco/event_detection/EventDetector.hpp"

#define EVENT_DEBUG 0

namespace loco {

  EventDetector::EventDetector(): toleratedDelay_(0.1)
  {

  } // constructor


  EventDetector::~EventDetector() {

  } // destructor


  bool EventDetector::initialize(double dt) {
    return true;
  } // initialize


  bool EventDetector::advance(double dt, loco::LegGroup& legs) {
    int iLeg = 0;
    for (auto leg : legs) {
      const double swingPhase  = leg->getSwingPhase();
      const double stancePhase = leg->getStancePhase();

      #if EVENT_DEBUG
      std::cout << *leg << std::endl;
      #endif

  //    if (leg->isInStanceMode() != leg->wasInStanceMode()) {
  //      std::cout << leg->getName() << " -----------------------------------------------------------------\n";
  //    }
  //
  //    if ((swingPhase >= 0.0 && swingPhase <= 0.5) && leg->wasInStanceMode()) {

      /* detect a lift off */
      if (leg->wasInStanceMode() && leg->isInSwingMode()) {
        // possible lift-off
        leg->getStateLiftOff()->setFootPositionInWorldFrame(leg->getWorldToFootPositionInWorldFrame()); // or base2foot?
        leg->getStateLiftOff()->setHipPositionInWorldFrame(leg->getWorldToHipPositionInWorldFrame());
  //      std::cout << leg->getName() << ": lift-off" << std::endl;
        leg->getStateLiftOff()->setIsNow(true);
      } else {
        leg->getStateLiftOff()->setIsNow(false);
      }

      /* detect a touch down */
      /*
      if (leg->wasInSwingMode() && leg->isInStanceMode()) {
        // possible touch-down
        std::cout << "touch down on leg:" << iLeg << std::endl;
        leg->getStateTouchDown()->setIsNow(true);
      } else {
        leg->getStateTouchDown()->setIsNow(false);
      }
      */

      //if (leg->wasInSwingMode() && leg->isGrounded()) {
      if ( !leg->wasGrounded() && leg->isGrounded() ) {
        leg->getStateTouchDown()->setIsNow(false);

        // A touchdown was detected, now check if it is earlier or later than expected
        if ( leg->isInSwingMode() && (swingPhase < (1-toleratedDelay_)) ) {
          #if EVENT_DEBUG
          std::cout << "[eventDetector] EARLY touchdown on leg: " << iLeg << std::endl;
          #endif
          leg->getStateTouchDownEarly()->setIsNow(true);
          leg->getStateTouchDownLate()->setIsNow(false);
        }
        else if ( leg->isInStanceMode() && (stancePhase > toleratedDelay_) ) {
          #if EVENT_DEBUG
          std::cout << "[eventDetector] LATE touchdown on leg: " << iLeg << std::endl;
          #endif
          leg->getStateTouchDownEarly()->setIsNow(false);
          leg->getStateTouchDownLate()->setIsNow(true);
        }
        else {
          #if EVENT_DEBUG
          std::cout << "[eventDetector] TOLERATED touchdown on leg: " << iLeg << std::endl;
          #endif
        }

      } // if touchdown
      else {
        leg->getStateTouchDown()->setIsNow(false);
      }

      iLeg++;
    } // for auto leg

    return true;

  } // advance


} /* namespace loco */
