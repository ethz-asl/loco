/*
 * StateSwitcher.hpp
 *
 *  Created on: Oct 5, 2014
 *      Author: dario
 */

#ifndef LOCO_STATESWITCHER_HPP_
#define LOCO_STATESWITCHER_HPP_

#include "loco/state_switcher/StateSwitcherBase.hpp"
#include <string>

namespace loco {


class StateSwitcher: public StateSwitcherBase {
 public:

  /* state_
   * 0: stance phase normal condition
   * 1: swing phase normal condition
   * 2: stance, but slipping
   * 3: stance, but lost contact / not yet touch-down
   * 4: swing, but late lift-off
   * 5: late swing, but early touch-down
   * 6: middle swing, but bumped into obstacle while swinging
   */
  enum States {
    Init,
    StanceNormal,
    StanceSlipping,
    StanceLostContact,
    SwingNormal,
    SwingLateLiftOff,
    SwingEarlyTouchDown,
    SwingBumpedIntoObstacle
  };

  StateSwitcher();
  virtual ~StateSwitcher();

  virtual bool initialize(double dt);

  virtual void setState(States state);
  virtual States getState();

 protected:
  States currentState_;
  States oldState_;

  virtual std::string getStateName(States state);

};


} /* namespace loco */

#endif /* LOCO_STATESWITCHER_HPP_ */
