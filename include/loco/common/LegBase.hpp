/*
 * LegBase.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_LEGBASE_HPP_
#define LOCO_LEGBASE_HPP_

#include "loco/common/LegStateLiftOff.hpp"
#include "loco/common/LegStateTouchDown.hpp"

namespace loco {

class LegBase {
 public:
  LegBase();
  virtual ~LegBase();

  virtual double getStancePhase();
  virtual double getSwingPhase();

  virtual double getStanceDuration();
  virtual double getSwingDuration();

  virtual bool isInStanceMode();
  virtual bool isInSwingMode();

  virtual bool isGrounded();
  virtual bool shouldBeGrounded();
  virtual bool isAndShoulBeGrounded();


  virtual void setStancePhase(double phase);
  virtual void setSwingPhase(double phase);

  virtual void setStanceDuration(double duration);
  virtual void setSwingDuration(double duration);

  virtual void setIsInStanceMode(bool isInStanceMode);
  virtual void setIsInSwingMode(bool isInSwingMode);

  virtual void setIsGrounded(bool isGrounded);
  virtual void setShouldBeGrounded(bool shouldBeGrounded);

  LegStateTouchDown* getStateTouchDown();
  LegStateLiftOff* getStateLiftOff();

 protected:
  double stancePhase_;
  double swingPhase_;
  double stanceDuration_;
  double swingDuration_;
  bool isInStanceMode_;
  bool isInSwingMode_;
  bool isGrounded_;
  bool shouldBeGrounded_;

  LegStateTouchDown stateTouchDown_;
  LegStateLiftOff stateLiftOff_;
};

} /* namespace loco */

#endif /* LOCO_LEGBASE_HPP_ */
