/*
 * ContactDetectorBase.hpp
 *
 *  Created on: May 26, 2014
 *      Author: gech
 */

#ifndef LOCO_CONTACTDETECTORBASE_HPP_
#define LOCO_CONTACTDETECTORBASE_HPP_

#include "loco/common/LegBase.hpp"


namespace loco {

class ContactDetectorBase {
 public:
  ContactDetectorBase();
  virtual ~ContactDetectorBase();

  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;
};

} /* namespace loco */

#endif /* CONTACTDETECTORBASE_HPP_ */
