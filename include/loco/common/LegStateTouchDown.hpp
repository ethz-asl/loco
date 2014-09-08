/*
 * LegStateTouchDown.hpp
 *
 *  Created on: Feb 26, 2014
 *      Author: gech
 */

#ifndef LOCO_LEGSTATETOUCHDOWN_HPP_
#define LOCO_LEGSTATETOUCHDOWN_HPP_

#include "loco/common/TypeDefs.hpp"
#include "loco/common/LegStateBase.hpp"

namespace loco {

//!  State of the leg at the event of touch-down
/*!
 */
class LegStateTouchDown : public loco::LegStateBase {
 public:
  LegStateTouchDown();
  virtual ~LegStateTouchDown();

  /*! Save foot position in world frame on touchdown event.
   * @param[in] footPositionInWorldFrame Measured foot position in world frame
   */
  void setTouchdownFootPositionInWorldFrame(const loco::Position& footPositionInWorldFrame);

  /*! Get the last touchdown foot position in world frame.
   */
  const Position& getFootPositionInWorldFrame() const;


 protected:
  loco::Position footPositionInWorldFrame_;

};

} /* namespace loco */

#endif /* LOCO_LEGSTATETOUCHDOWN_HPP_ */
