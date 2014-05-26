/*
 * LegStateBase.hpp
 *
 *  Created on: Feb 26, 2014
 *      Author: gech
 */

#ifndef LOCO_LEGSTATEBASE_HPP_
#define LOCO_LEGSTATEBASE_HPP_

namespace loco {


//! Base class for state of the Leg
class LegStateBase {
 public:
  LegStateBase();
  virtual ~LegStateBase();

  bool isNow() const;
  void setIsNow(bool isNow);

 protected:
  bool isNow_;
};

} /* namespace loco */

#endif /* LEGSTATEBASE_HPP_ */
