/*
 * TuningWithSliderboard.hpp
 *
 *  Created on: Mar 31, 2014
 *      Author: gech
 */

#ifndef LOCO_TUNINGWITHSLIDERBOARD_HPP_
#define LOCO_TUNINGWITHSLIDERBOARD_HPP_

#include "Sliderboard.hpp"

namespace loco {

class TuningWithSliderboard {
 public:
  TuningWithSliderboard(robotUtils::Sliderboard* sliderboard);
  virtual ~TuningWithSliderboard();
  virtual bool initialize(double dt);
  virtual bool advance(double dt);
 protected:
  //! MIDI board Commands used for parameter tuning
  robotUtils::Sliderboard* sliderboard_;
};

} /* namespace loco */

#endif /* LOCO_TUNINGWITHSLIDERBOARD_HPP_ */
