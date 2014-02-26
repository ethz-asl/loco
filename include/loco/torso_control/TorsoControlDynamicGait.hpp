/*!
* @file     BaseControlDynamicGait.hpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#ifndef LOCO_BASECONTROLDYNAMICGAIT_HPP_
#define LOCO_BASECONTROLDYNAMICGAIT_HPP_

#include "loco/torso_control/TorsoControlBase.hpp"

namespace loco {

class TorsoControlDynamicGait: public TorsoControlBase {
 public:
  TorsoControlDynamicGait();
  virtual ~TorsoControlDynamicGait();

  virtual void advance(double dt);
};

} /* namespace loco */

#endif /* LOCO_BASECONTROLDYNAMICGAIT_HPP_ */
