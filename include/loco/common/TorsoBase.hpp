/*
 * StateBase.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_STATEBASE_HPP_
#define LOCO_STATEBASE_HPP_


#include "loco/common/TypeDefs.hpp"

#include "loco/common/TorsoStateDesired.hpp"
#include "loco/common/TorsoStateMeasured.hpp"

namespace loco {

class TorsoBase {
 public:
  typedef loco::Twist Twist;
  typedef loco::Pose Pose;
  typedef loco::Position Position;
 public:
  TorsoBase();
  virtual ~TorsoBase();

  virtual TorsoStateMeasured& getMeasuredState() = 0;
  virtual TorsoStateDesired& getDesiredState() = 0;

  virtual double getStridePhase() = 0;
  virtual void setStridePhase(double stridePhase) = 0;












  virtual void advance(double dt) = 0;

};

} /* namespace loco */

#endif /* STATEBASE_HPP_ */
