/*
 * LegStateTouchDown.cpp
 *
 *  Created on: Feb 26, 2014
 *      Author: gech
 */

#include "loco/common/LegStateTouchDown.hpp"

namespace loco {

  LegStateTouchDown::LegStateTouchDown() :
    LegStateBase(),
    footPositionInWorldFrame_()
  {


  }


  LegStateTouchDown::~LegStateTouchDown() {

  }


  void LegStateTouchDown::setTouchdownFootPositionInWorldFrame(const loco::Position& footPositionInWorldFrame) {
    footPositionInWorldFrame_ = footPositionInWorldFrame;
  }


  const Position& LegStateTouchDown::getFootPositionInWorldFrame() const {
    return footPositionInWorldFrame_;
  }


} /* namespace loco */
