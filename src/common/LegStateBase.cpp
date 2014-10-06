/*
 * LegStateBase.cpp
 *
 *  Created on: Feb 26, 2014
 *      Author: gech
 */

#include "loco/common/LegStateBase.hpp"

namespace loco {

  LegStateBase::LegStateBase() :
        isNow_(false),
        lastStateWasEarly_(false),
        lastStateWasLate_(false),
        stateChangedAtTime_(0.0),
        swingPhase_(0.0)
  {


  }


  LegStateBase::~LegStateBase() {

  }


  void LegStateBase::setIsNow(bool isNow) { isNow_= isNow; }
  bool LegStateBase::isNow() const { return isNow_; }

  void LegStateBase::setLastStateWasEarly(bool wasEarly) { lastStateWasEarly_ = wasEarly; }
  bool LegStateBase::lastStateWasEarly() const { return lastStateWasEarly_; }

  void LegStateBase::setLastStateWasLate(bool wasLate) { lastStateWasLate_ = wasLate; }
  bool LegStateBase::lastStateWasLate() const { return lastStateWasLate_; }

  void LegStateBase::setStateChangedAtTime(double time) { stateChangedAtTime_ = time; }
  double LegStateBase::StateChangedAtTime() const { return stateChangedAtTime_; }

  double LegStateBase::getSwingPhase() const {
    return swingPhase_;
  }

  void LegStateBase::setSwingPhase(double swingPhase) {
    swingPhase_ = swingPhase;
  }


} /* namespace loco */
