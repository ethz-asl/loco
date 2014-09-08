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
#define MINIMUM_SPEED_FOR_SLIP_DETECTION    0.01  // m/s
#define TIME_DELAY_FOR_SLIP_DEBUG_PRINTOUT  0.5   // s

namespace loco {

  EventDetector::EventDetector():
      EventDetectorBase(),
      toleratedDelay_(DEFAULT_EVENT_DELAY_TOLERANCE),
      timeSinceLastPrintout_(4)
  {

  } // constructor


  EventDetector::~EventDetector() {

  } // destructor


  bool EventDetector::initialize(double dt) {
    return true;
  } // initialize

  int counter[4] = {0,0,0,0};

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
      if ( leg->isGrounded() ) {
        /* It is assumed that the foot can slip only if grounded. Check if the distance between
         * the current foot position and the touchdown foot position in world frame if greater
         * than a default minimum.
         */

        loco::LinearVelocity footVelocityInWorldFrame = leg->getFootLinearVelocityInWorldFrame();
        loco::Position distanceFromTouchdown = leg->getWorldToFootPositionInWorldFrame()
                                               - leg->getStateTouchDown()->getFootPositionInWorldFrame();

        if ( (distanceFromTouchdown.norm() > MINIMUM_DISTANCE_FOR_SLIP_DETECTION) ) {
          leg->setIsSlipping(true);

          #if EVENT_DEBUG
          if ( timeSinceLastPrintout_[iLeg] == 0.0 ) {
            std::cout << "[eventDetector] leg "       << iLeg << " is slipping!" << std::endl;
            std::cout << "speed: "                    << footVelocityInWorldFrame.norm() << std::endl;
            std::cout << "distance from touchdown: "  << distanceFromTouchdown.norm() << std::endl;
          }

          timeSinceLastPrintout_[iLeg] += dt;

          if ( (timeSinceLastPrintout_[iLeg] > TIME_DELAY_FOR_SLIP_DEBUG_PRINTOUT) ) {
            timeSinceLastPrintout_[iLeg] = 0.0;
          }
          #endif

        } // if slipping
        else {
          /* Slipping state should be reset if the foot has not traveled for MINIMUM_DISTANCE_FOR_SLIP_DETECTION
           * or if, after slipping for some time, its speed drops below MINIMUM_SPEED_FOR_SLIP_DETECTION
           */
          if ( (footVelocityInWorldFrame.norm() < MINIMUM_SPEED_FOR_SLIP_DETECTION) ) {
            leg->setIsSlipping(false);
          }
        }
      } // if grounded
      else {
        // foot cannot be slipping if not grounded
        leg->setIsSlipping(false);
        #if EVENT_DEBUG
        timeSinceLastPrintout_[iLeg] = 0.0;
        #endif
      }
      /**********************
       * End slip detection *
       **********************/

      if (leg->isSlipping()) {
        counter[iLeg]++;
        std::cout << "leg: " << iLeg << " isSlipping. counter: " << counter[iLeg] << std::endl;
      }
      else {
        counter[iLeg] = 0;
      }

      iLeg++;
    } // for auto leg

    return true;

  } // advance


} /* namespace loco */
