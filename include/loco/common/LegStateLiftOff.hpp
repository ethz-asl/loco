/*
 * LegStateLiftOff.hpp
 *
 *  Created on: Feb 26, 2014
 *      Author: gech
 */

#ifndef LOCO_LEGSTATELIFTOFF_HPP_
#define LOCO_LEGSTATELIFTOFF_HPP_

#include "loco/common/TypeDefs.hpp"
#include "loco/common/LegStateBase.hpp"

#include <Eigen/Core>

namespace loco {

//!  State of the leg at the event of lift-off
/*!
 */
class LegStateLiftOff : public loco::LegStateBase {
 public:
  LegStateLiftOff();
  virtual ~LegStateLiftOff();

  const Position& getHipPositionInWorldFrame() const;
  const Position& getFootPositionInWorldFrame() const;

  void setHipPositionInWorldFrame(const Position& hipPositionInWorldFrame);
  void setFootPositionInWorldFrame(const Position& footPositionInWorldFrame);



 protected:
  Position footPositionInWorldFrame_;
  Position hipPositionInWorldFrame_;

};

} /* namespace loco */

#endif /* LOCO_LEGSTATELIFTOFF_HPP_ */
