/*
 * LegStateTouchDown.hpp
 *
 *  Created on: Feb 26, 2014
 *      Author: gech
 */

#ifndef LOCO_LEGSTATETOUCHDOWN_HPP_
#define LOCO_LEGSTATETOUCHDOWN_HPP_

#include "loco/common/LegStateBase.hpp"

namespace loco {

//!  State of the leg at the event of touch-down
/*!
 */
class LegStateTouchDown : public loco::LegStateBase {
 public:
  LegStateTouchDown();
  virtual ~LegStateTouchDown();

};

} /* namespace loco */

#endif /* LOCO_LEGSTATETOUCHDOWN_HPP_ */
