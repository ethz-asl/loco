/*
 * ContactDetectorConstantDuringStance.hpp
 *
 *  Created on: May 26, 2014
 *      Author: gech
 */

#ifndef LOCO_CONTACTDETECTORCONSTANTDURINGSTANCE_HPP_
#define LOCO_CONTACTDETECTORCONSTANTDURINGSTANCE_HPP_

#include "loco/contact_detection/ContactDetectorBase.hpp"
#include "loco/common/LegGroup.hpp"
namespace loco {

class ContactDetectorConstantDuringStance: public ContactDetectorBase {
 public:
  ContactDetectorConstantDuringStance(LegGroup* legs);
  virtual ~ContactDetectorConstantDuringStance();
  virtual bool initialize(double dt);
  virtual bool advance(double dt);

 protected:
  LegGroup* legs_;

  bool registeredContact_[4];

};

} /* namespace loco */

#endif /* CONTACTDETECTORCONSTANTDURINGSTANCE_HPP_ */
