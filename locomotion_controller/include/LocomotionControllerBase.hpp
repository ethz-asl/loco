/*!
* @file     LocomotionControllerBase.hpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#ifndef LOCO_LOCOMOTIONCONTROLLERBASE_HPP_
#define LOCO_LOCOMOTIONCONTROLLERBASE_HPP_


namespace loco {

class LocomotionControllerBase {
 public:
  LocomotionControllerBase();
  virtual ~LocomotionControllerBase();

  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual void advance(double dt) = 0;
 protected:

};

} /* namespace loco */

#endif /* LOCOMOTIONCONTROLLERBASE_HPP_ */
