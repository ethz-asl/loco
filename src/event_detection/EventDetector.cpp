/*
 * EventDetector.cpp
 *
 *  Created on: Sep 5, 2014
 *      Author: C. Dario Bellicoso
 */


#include "loco/event_detection/EventDetector.hpp"

#define EVENT_DEBUG                         0
#define DEFAULT_EVENT_DELAY_TOLERANCE       0.1
#define MINIMUM_DISTANCE_FOR_SLIP_DETECTION 0.01  // m
#define TIME_DELAY_FOR_SLIP_DEBUG_PRINTOUT  0.5   // s

namespace loco {

  EventDetector::EventDetector():
      EventDetectorBase(),
      toleratedDelay_(DEFAULT_EVENT_DELAY_TOLERANCE),
      timeSinceLastPrintout_(0.0)
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
      //std::cout << *leg << std::endl;
      #endif

      /*********************
       * Liftoff detection *
       *********************/
      if (leg->wasGrounded() && !leg->isGrounded()) {
        leg->getStateLiftOff()->setFootPositionInWorldFrame(leg->getWorldToFootPositionInWorldFrame()); // or base2foot?
        leg->getStateLiftOff()->setHipPositionInWorldFrame(leg->getWorldToHipPositionInWorldFrame());
        leg->getStateLiftOff()->setIsNow(true);
        // A liftoff was detected, now check if it is earlier or later than expected
        if ( leg->isInStanceMode() && (stancePhase < (1-toleratedDelay_)) ) {
          #if EVENT_DEBUG
          std::cout << "[eventDetector] EARLY liftoff on leg: " << iLeg << std::endl;
          #endif
          leg->getStateLiftOffEarly()->setIsNow(true);
          leg->getStateLiftOffLate()->setIsNow(false);
        }
        else if ( leg->isInSwingMode() && (swingPhase > toleratedDelay_) ) {
          #if EVENT_DEBUG
          std::cout << "[eventDetector] LATE liftoff on leg: " << iLeg << std::endl;
          #endif
          leg->getStateLiftOffEarly()->setIsNow(false);
          leg->getStateLiftOffLate()->setIsNow(true);
        }
        else {
          #if EVENT_DEBUG
          std::cout << "[eventDetector] TOLERATED liftoff on leg: " << iLeg << std::endl;
          #endif
        }
      } // if liftoff
      else {
        // reset liftoff state
        leg->getStateLiftOff()->setIsNow(false);
      }
      /*************************
       * End liftoff detection *
       *************************/

      /***********************
       * Touchdown detection *
       ***********************/
      if ( !leg->wasGrounded() && leg->isGrounded() ) {
        leg->getStateTouchDown()->setIsNow(true);
        leg->getStateTouchDown()->setTouchdownFootPositionInWorldFrame(leg->getWorldToFootPositionInWorldFrame());
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
        // reset touchdown state
        leg->getStateTouchDown()->setIsNow(false);
      }
      /***************************
       * End touchdown detection *
       ***************************/

      /******************
       * Slip detection *
       ******************/
      loco::LinearVelocity footVelocityInWorldFrame = leg->getFootLinearVelocityInWorldFrame();

      if ( leg->isGrounded() ) {
        /* Check if the distance between the current foot position and the touchdown foot position
         * in world frame if greater than a default minimum.
         */
        loco::Position distanceFromTouchdown = leg->getWorldToFootPositionInWorldFrame()
                                               - leg->getStateTouchDown()->getFootPositionInWorldFrame();

        if (distanceFromTouchdown.norm() > MINIMUM_DISTANCE_FOR_SLIP_DETECTION) {
          leg->setIsSlipping(true);

          #if EVENT_DEBUG
          if ( timeSinceLastPrintout_ == 0.0 ) {
            std::cout << "[eventDetector] leg "       << iLeg << " is slipping!" << std::endl;
            std::cout << "speed: "                    << footVelocityInWorldFrame.norm() << std::endl;
            std::cout << "distance from touchdown: "  << distanceFromTouchdown.norm() << std::endl;
          }

          timeSinceLastPrintout_ += dt;

          if ( (timeSinceLastPrintout_ > TIME_DELAY_FOR_SLIP_DEBUG_PRINTOUT) ) {
            timeSinceLastPrintout_ = 0.0;
          }
          #endif

        } // if slipping
        else {
          // foot cannot be slipping if in air
          leg->setIsSlipping(false);
        }

      }
      /**********************
       * End slip detection *
       **********************/

      iLeg++;
    } // for auto leg

    return true;

  } // advance


} /* namespace loco */
