/*
 * TuningWithSliderboard.hpp
 *
 *  Created on: Mar 31, 2014
 *      Author: gech
 */

#ifndef LOCO_TUNINGWITHSLIDERBOARDBASE_HPP_
#define LOCO_TUNINGWITHSLIDERBOARDBASE_HPP_

#include "Sliderboard.hpp"

namespace loco {

class TuningWithSliderboardBase {
 public:
  TuningWithSliderboardBase(robotUtils::Sliderboard* sliderboard);
  virtual ~TuningWithSliderboardBase();
  virtual bool initialize(double dt);
  virtual bool advance(double dt);
 protected:
  //! MIDI board Commands used for parameter tuning
  robotUtils::Sliderboard* sliderboard_;
};

} /* namespace loco */

#endif /* LOCO_TUNINGWITHSLIDERBOARDBASE_HPP_ */
