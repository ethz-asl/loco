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

  /*!
   * Initializes locomotion controller
   * @param dt the time step [s]
   * @return true if successfull.
   */
  virtual bool initialize(double dt) = 0;

  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual void advance(double dt) = 0;
 protected:

};

} /* namespace loco */

#endif /* LOCOMOTIONCONTROLLERBASE_HPP_ */
