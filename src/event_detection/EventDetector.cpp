/*
 * EventDetector.cpp
 *
 *  Created on: Sep 5, 2014
 *      Author: C. Dario Bellicoso
 */


#include "loco/event_detection/EventDetector.hpp"

#define EVENT_DEBUG   0

namespace loco {

  EventDetector::EventDetector():
      EventDetectorBase(),
      toleratedDelay_(0.1),
      minimumDistanceForSlipDetection_(0.01),
      minimumSpeedForSlipDetection_(0.01),
      timeSinceInit_(0.0)
  {

  } // constructor


  EventDetector::~EventDetector() {

  } // destructor


  bool EventDetector::initialize(double dt) {
    toleratedDelay_ = 0.1;
    minimumDistanceForSlipDetection_ = 0.01;
    minimumSpeedForSlipDetection_ = 0.01;
    timeSinceInit_ = 0.0;

    return true;
  } // initialize


  bool EventDetector::advance(double dt, loco::LegGroup& legs) {
    timeSinceInit_ += dt;
    int iLeg = 0;

    for (auto leg : legs) {
      const double swingPhase  = leg->getSwingPhase();
      const double stancePhase = leg->getStancePhase();

      // Reset properties at start of stance phase
      if ( (leg->getPreviousStancePhase()==-1) && (leg->getStancePhase()>= 0.0) ) {
    	  // Begininnig of stance phase
    	  leg->setDidTouchDownAtLeastOnceDuringStance(false);
      }

      /*********************
       * Liftoff detection *
       *********************/
      if (leg->wasGrounded() && !leg->isGrounded()) {
        leg->getStateLiftOff()->setFootPositionInWorldFrame(leg->getWorldToFootPositionInWorldFrame()); // or base2foot?
        leg->getStateLiftOff()->setHipPositionInWorldFrame(leg->getWorldToHipPositionInWorldFrame());
        leg->getStateLiftOff()->setIsNow(true);
        leg->getStateLiftOff()->setStateChangedAtTime(timeSinceInit_);
        // A liftoff was detected, now check if it is earlier or later than expected
        if ( leg->shouldBeGrounded() && (stancePhase < (1-toleratedDelay_)) ) {
          #if EVENT_DEBUG
          std::cout << "[eventDetector] EARLY liftoff on leg: " << iLeg << std::endl;
          #endif
          leg->getStateLiftOff()->setLastStateWasEarly(true);
          leg->getStateLiftOff()->setLastStateWasLate(false);
        }
        else if ( !leg->shouldBeGrounded() && (swingPhase > toleratedDelay_) ) {
          #if EVENT_DEBUG
          std::cout << "[eventDetector] LATE liftoff on leg: " << iLeg << std::endl;
          #endif
          leg->getStateLiftOff()->setLastStateWasEarly(false);
          leg->getStateLiftOff()->setLastStateWasLate(true);
        }
        else {
          #if EVENT_DEBUG
          std::cout << "[eventDetector] TOLERATED liftoff on leg: " << iLeg << std::endl;
          #endif
          leg->getStateLiftOff()->setLastStateWasEarly(false);
          leg->getStateLiftOff()->setLastStateWasLate(false);
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

        if (!leg->didTouchDownAtLeastOnceDuringStance()) {
        	leg->setDidTouchDownAtLeastOnceDuringStance(true);
        }

        leg->getStateTouchDown()->setTouchdownFootPositionInWorldFrame(leg->getWorldToFootPositionInWorldFrame());
        leg->getStateTouchDown()->setStateChangedAtTime(timeSinceInit_);
        // A touchdown was detected, now check if it is earlier or later than expected
        if ( !leg->shouldBeGrounded() && (swingPhase < (1-toleratedDelay_)) ) {
          #if EVENT_DEBUG
          std::cout << "[eventDetector] EARLY touchdown on leg: " << iLeg << std::endl;
          #endif
          leg->getStateTouchDown()->setLastStateWasEarly(true);
          leg->getStateTouchDown()->setLastStateWasLate(false);
        }
        else if ( leg->shouldBeGrounded() && (stancePhase > toleratedDelay_) ) {
          #if EVENT_DEBUG
          std::cout << "[eventDetector] LATE touchdown on leg: " << iLeg << std::endl;
          #endif
          leg->getStateTouchDown()->setLastStateWasEarly(false);
          leg->getStateTouchDown()->setLastStateWasLate(true);
        }
        else {
          #if EVENT_DEBUG
          std::cout << "[eventDetector] TOLERATED touchdown on leg: " << iLeg << std::endl;
          #endif
          leg->getStateTouchDown()->setLastStateWasEarly(false);
          leg->getStateTouchDown()->setLastStateWasLate(false);
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
         * the current foot position and the touchdown foot position in world frame is greater
         * than a default minimum. Slipping stops when the linear velocity drops in norm under
         * a default minimum.
         */
        loco::LinearVelocity footVelocityInWorldFrame = leg->getFootLinearVelocityInWorldFrame();
        loco::Position distanceFromTouchdown = leg->getWorldToFootPositionInWorldFrame()
                                               - leg->getStateTouchDown()->getFootPositionInWorldFrame();

        if ( (distanceFromTouchdown.norm() > minimumDistanceForSlipDetection_) ) {
          leg->setIsSlipping(true);

          #if EVENT_DEBUG
          std::cout << "[eventDetector] leg "       << iLeg << " is slipping!" << std::endl;
          std::cout << "speed: "                    << footVelocityInWorldFrame.norm() << std::endl;
          std::cout << "distance from touchdown: "  << distanceFromTouchdown.norm() << std::endl;
          #endif

        } // if slipping

        /* Slipping state should be reset if the foot has not traveled for MINIMUM_DISTANCE_FOR_SLIP_DETECTION
         * or if, after slipping for some time, its speed drops below MINIMUM_SPEED_FOR_SLIP_DETECTION
         */
        if ( (footVelocityInWorldFrame.norm() < minimumSpeedForSlipDetection_) ) {
          leg->setIsSlipping(false);
        }
      } // if grounded
      else {
        // foot cannot be slipping if not grounded
        leg->setIsSlipping(false);
      }
      /**********************
       * End slip detection *
       **********************/


      /***************************
       * Check if losing contact during stance phase when leg is supposed to be grounded *
       ***************************/
      if (!leg->isGrounded() && leg->shouldBeGrounded() ) {
          if ( leg->didTouchDownAtLeastOnceDuringStance() ) {
        	  leg->setIsLosingContact(true);
          }
      }
      else {
    	  leg->setIsLosingContact(false);
      }
      /*******************************
       * End check if losing contact *
       *******************************/


      iLeg++;
    } // for auto leg

    return true;

  } // advance

} /* namespace loco */
