/*
 * EventDetector.hpp
 *
 *  Created on: Sep 5, 2014
 *      Author: C. Dario Bellicoso
 */

#ifndef LOCO_EVENTDETECTOR_HPP_
#define LOCO_EVENTDETECTOR_HPP_

#include "loco/event_detection/EventDetectorBase.hpp"
#include "loco/common/LegGroup.hpp"

namespace loco {

  class EventDetector: public EventDetectorBase {
   public:
    EventDetector();
    virtual ~EventDetector();

    virtual bool initialize(double dt);
    virtual bool advance(double dt, loco::LegGroup& legs);

   protected:
    double toleratedDelay_;
    double minimumDistanceForSlipDetection_;
    double minimumSpeedForSlipDetection_;
    double timeSinceInit_;
  };

} /* namespace loco */

#endif /* LOCO_EVENTDETECTOR_HPP_ */
