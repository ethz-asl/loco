/*
 * TuningWithSliderboard.cpp
 *
 *  Created on: Mar 31, 2014
 *      Author: gech
 */

#include "loco/tools/TuningWithSliderboardBase.hpp"

namespace loco {

TuningWithSliderboardBase::TuningWithSliderboardBase(robotUtils::Sliderboard* sliderboard) :
  sliderboard_(sliderboard)
{


}

TuningWithSliderboardBase::~TuningWithSliderboardBase() {

}

bool TuningWithSliderboardBase::initialize(double dt)
{


  return true;
}

bool TuningWithSliderboardBase::advance(double dt) {
  return true;
}

} /* namespace loco */
