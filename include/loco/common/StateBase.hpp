/*
 * StateBase.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_STATEBASE_HPP_
#define LOCO_STATEBASE_HPP_

namespace loco {

class StateBase {
 public:
  StateBase();
  virtual ~StateBase();

  virtual double getHeadingSpeed() = 0;
  virtual double getTurningSpeed() = 0;
  virtual double getLateralSpeed() = 0;
};

} /* namespace loco */

#endif /* STATEBASE_HPP_ */
