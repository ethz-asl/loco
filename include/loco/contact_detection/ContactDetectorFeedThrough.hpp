/*
 * ContactDetectorFeedThrough.h
 *
 *  Created on: May 26, 2014
 *      Author: gech
 */

#ifndef LOCO_CONTACTDETECTORFEEDTHROUGH_H_
#define LOCO_CONTACTDETECTORFEEDTHROUGH_H_

#include "loco/contact_detection/ContactDetectorBase.hpp"

namespace loco {

class ContactDetectorFeedThrough: public ContactDetectorBase {
 public:
  ContactDetectorFeedThrough();
  virtual ~ContactDetectorFeedThrough();

  virtual bool initialize(double dt);
  virtual bool advance(double dt);
};

} /* namespace loco */

#endif /* CONTACTDETECTORFEEDTHROUGH_H_ */
