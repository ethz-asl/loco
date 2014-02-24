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

#include "BaseControlBase.hpp"

namespace loco {

class BaseControlDynamicGait: public BaseControlBase {
 public:
  BaseControlDynamicGait();
  virtual ~BaseControlDynamicGait();
};

} /* namespace loco */

#endif /* LOCO_BASECONTROLDYNAMICGAIT_HPP_ */
