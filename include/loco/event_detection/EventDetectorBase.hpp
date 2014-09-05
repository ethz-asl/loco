/*
 * EventDetectorBase.hpp
 *
 *  Created on: Sep 5, 2014
 *      Author: C. Dario Bellicoso
 */

#ifndef LOCO_EVENTDETECTORBASE_HPP_
#define LOCO_EVENTDETECTORBASE_HPP_

#include "loco/common/LegGroup.hpp"

namespace loco {

  class EventDetectorBase {
   public:
    EventDetectorBase();
    virtual ~EventDetectorBase();

    virtual bool initialize(double dt);
    virtual bool advance(double dt, loco::LegGroup& legs);

   protected:
  };

} /* namespace loco */

#endif /* LOCO_EVENTDETECTORBASE_HPP_ */
