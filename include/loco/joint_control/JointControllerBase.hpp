/*
 * JointControllerBase.hpp
 *
 *  Created on: Apr 2, 2014
 *      Author: gech
 */

#ifndef LOCO_JOINTCONTROLLERBASE_HPP_
#define LOCO_JOINTCONTROLLERBASE_HPP_

namespace loco {

class JointControllerBase {
 public:
  JointControllerBase();
  virtual ~JointControllerBase();

  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;
};

} /* namespace loco */

#endif /* LOCO_JOINTCONTROLLERBASE_HPP_ */
