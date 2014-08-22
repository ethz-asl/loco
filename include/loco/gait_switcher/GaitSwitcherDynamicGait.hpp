/*
 * GaitSwitcherDynamicGait.hpp
 *
 *  Created on: Aug 22, 2014
 *      Author: gech
 */

#ifndef LOCO_GAITSWITCHERDYNAMICGAIT_HPP_
#define LOCO_GAITSWITCHERDYNAMICGAIT_HPP_

#include "loco/gait_switcher/GaitSwitcherBase.hpp"

namespace loco {

class GaitSwitcherDynamicGait: public GaitSwitcherBase {
 public:
  GaitSwitcherDynamicGait();
  virtual ~GaitSwitcherDynamicGait();

  virtual bool initialize(double dt);
  virtual bool advance(double dt);
};

} /* namespace loco */

#endif /* GAITSWITCHERDYNAMICGAIT_HPP_ */
