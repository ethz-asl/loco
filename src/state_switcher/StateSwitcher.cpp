/*
 * StateSwitcher.cpp
 *
 *  Created on: Oct 5, 2014
 *      Author: dario
 */

#include "loco/state_switcher/StateSwitcher.hpp"
#include <iostream>

const bool STATESWITCHER_DEBUG = true;

namespace loco {

StateSwitcher::StateSwitcher(): StateSwitcherBase() {
  currentState_ = States::Init;
  oldState_ = States::Init;
}


StateSwitcher::~StateSwitcher() {

}


bool StateSwitcher::initialize(double dt) {
  currentState_ = States::Init;
  oldState_ = States::Init;

  return true;
}


StateSwitcher::States StateSwitcher::getState() {
  return currentState_;
}


void StateSwitcher::setState(States state) {
  if (STATESWITCHER_DEBUG) {
    std::cout << "Setting state from: " << getStateName(currentState_) << " to: " << getStateName(state) << std::endl;
  }
  oldState_ = currentState_;
  currentState_ = state;
}


std::string StateSwitcher::getStateName(StateSwitcher::States state) {
  switch(state) {
    case(States::Init):                     return std::string("Init");
    case(States::StanceNormal):             return std::string("StanceNormal");
    case(States::StanceSlipping):           return std::string("StanceSlipping");
    case(States::StanceLostContact):        return std::string("StanceLostContact");
    case(States::SwingNormal):              return std::string("SwingNormal");
    case(States::SwingLateLiftOff):         return std::string("SwingLateLiftOff");
    case(States::SwingEarlyTouchDown):      return std::string("SwingEarlyTouchDown");
    case(States::SwingBumpedIntoObstacle):  return std::string("SwingBumpedIntoObstacle");
    default:                                return std::string("unknown state");
  }
}

} /* namespace loco */

