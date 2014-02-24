/*!
* @file     LocomotionControllerDynamicGait.hpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#ifndef LOCO_LOCOMOTIONCONTROLLERDYNAMICGAIT_HPP_
#define LOCO_LOCOMOTIONCONTROLLERDYNAMICGAIT_HPP_

#include "LocomotionControllerBase.hpp"

namespace loco {

class LocomotionControllerDynamicGait: public LocomotionControllerBase {
 public:
  LocomotionControllerDynamicGait();
  virtual ~LocomotionControllerDynamicGait();
};

} /* namespace loco */

#endif /* LOCO_LOCOMOTIONCONTROLLERDYNAMICGAIT_HPP_ */
