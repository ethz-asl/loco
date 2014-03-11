/*
 * TorsoStateMeasured.hpp
 *
 *  Created on: Mar 4, 2014
 *      Author: gech
 */

#ifndef LOCO_TORSOSTATEMEASURED_HPP_
#define LOCO_TORSOSTATEMEASURED_HPP_


#include "loco/common/TorsoStateBase.hpp"

namespace loco {


//! Measured state of the torso
class TorsoStateMeasured: public TorsoStateBase {
 public:
  TorsoStateMeasured();
  virtual ~TorsoStateMeasured();



};

} /* namespace loco */

#endif /* LOCO_TORSOSTATEMEASURED_HPP_ */
