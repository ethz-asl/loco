/*
 * GaitSwitcherBase.hpp
 *
 *  Created on: Aug 22, 2014
 *      Author: gech
 */

#ifndef LOCO_GAITSWITCHERBASE_HPP_
#define LOCO_GAITSWITCHERBASE_HPP_

namespace loco {

class GaitSwitcherBase {
 public:
  GaitSwitcherBase();
  virtual ~GaitSwitcherBase();

  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;
};

} /* namespace loco */

#endif /* GAITSWITCHERBASE_HPP_ */
