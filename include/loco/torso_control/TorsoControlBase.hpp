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

class TorsoControlBase {
 public:
  TorsoControlBase();
  virtual ~TorsoControlBase();
  virtual bool initialize(double dt) = 0;
  virtual void advance(double dt) = 0;
};

} /* namespace loco */

#endif /* BASECONTROLBASE_HPP_ */
