/*
 * TuningWithSliderboardLocoDynamicGait.hpp
 *
 *  Created on: Apr 1, 2014
 *      Author: gech
 */

#ifndef TUNINGWITHSLIDERBOARDLOCODYNAMICGAIT_HPP_
#define TUNINGWITHSLIDERBOARDLOCODYNAMICGAIT_HPP_

#include "loco/tools/TuningWithSliderboardBase.hpp"
#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"

namespace loco {

class TuningWithSliderboardLocoDynamicGait: public TuningWithSliderboardBase {
 public:
  TuningWithSliderboardLocoDynamicGait(robotUtils::Sliderboard* sliderboard);
  TuningWithSliderboardLocoDynamicGait(robotUtils::Sliderboard* sliderboard, LocomotionControllerDynamicGait* locomotionController);
  virtual ~TuningWithSliderboardLocoDynamicGait();
  virtual bool initialize(double dt);
  virtual bool advance(double dt);

  void setLocomotionController(LocomotionControllerDynamicGait* locomotionController);
 protected:
  LocomotionControllerDynamicGait* locomotionController_;

 protected:
  bool initializeVirtualModelController(double dt);
  bool advanceVirtualModelController(double dt);


};

} /* namespace loco */

#endif /* TUNINGWITHSLIDERBOARDLOCODYNAMICGAIT_HPP_ */
