/*
 * LegStateLiftOff.hpp
 *
 *  Created on: Feb 26, 2014
 *      Author: gech
 */

#ifndef LOCO_LEGSTATELIFTOFF_HPP_
#define LOCO_LEGSTATELIFTOFF_HPP_

#include "loco/common/LegStateBase.hpp"

#include <Eigen/Core>

namespace loco {

class LegStateLiftOff : public loco::LegStateBase {
 private:
  typedef Eigen::Vector3d Position;
 public:
  LegStateLiftOff();
  virtual ~LegStateLiftOff();

  const Position& getHipPositionInWorldFrame() const;
  const Position& getFootPositionInWorldFrame() const;

  void setHipPositionInWorldFrame(const LegStateLiftOff::Position& hipPositionInWorldFrame);
  void setFootPositionInWorldFrame(const LegStateLiftOff::Position& footPositionInWorldFrame);
 protected:
  Position footPositionInWorldFrame_;
  Position hipPositionInWorldFrame_;
};

} /* namespace loco */

#endif /* LOCO_LEGSTATELIFTOFF_HPP_ */
