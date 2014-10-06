/*
 * StateSwitcherBase.hpp
 *
 *  Created on: Oct 5, 2014
 *      Author: dario
 */

#ifndef LOCO_STATESWITCHERBASE_HPP_
#define LOCO_STATESWITCHERBASE_HPP_


namespace loco {

class StateSwitcherBase {
  public:

    StateSwitcherBase();
    virtual ~StateSwitcherBase();

    virtual bool initialize(double dt) = 0;

};

} /* namespace loco */


#endif /* LOCO_STATESWITCHERBASE_HPP_ */
