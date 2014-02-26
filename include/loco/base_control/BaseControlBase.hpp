/*!
* @file     BaseControlBase.hpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/

#ifndef LOCO_BASECONTROLBASE_HPP_
#define LOCO_BASECONTROLBASE_HPP_

namespace loco {

class BaseControlBase {
 public:
  BaseControlBase();
  virtual ~BaseControlBase();

  virtual void advance(double dt) = 0;
};

} /* namespace loco */

#endif /* BASECONTROLBASE_HPP_ */
