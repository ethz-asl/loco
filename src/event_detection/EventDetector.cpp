/*
 * EventDetector.cpp
 *
 *  Created on: Sep 5, 2014
 *      Author: C. Dario Bellicoso
 */


#include "loco/event_detection/EventDetector.hpp"

namespace loco {

  EventDetector::EventDetector() {
    std::cout << "*************constructing" << std::endl;
  } // constructor


  EventDetector::~EventDetector() {

  } // destructor


  bool EventDetector::initialize(double dt) {
    return true;
  } // initialize


  bool EventDetector::advance(double dt, loco::LegGroup& legs) {
    int iLeg = 0;
    for (auto leg : legs) {
      const double swingPhase = leg->getSwingPhase();
      std::cout << *leg << std::endl;

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
      if (leg->wasInSwingMode() && leg->isInStanceMode()) {
        // possible touch-down
        std::cout << "touch down on leg:" << iLeg << std::endl;
        leg->getStateTouchDown()->setIsNow(true);
      } else {
        leg->getStateTouchDown()->setIsNow(false);
      }

      iLeg++;
    } // for auto leg

    return true;

  } // advance


} /* namespace loco */
