/*
 * LegStateBase.cpp
 *
 *  Created on: Feb 26, 2014
 *      Author: gech
 */

#include "loco/common/LegStateBase.hpp"

namespace loco {

LegStateBase::LegStateBase() :
      isNow_(false)
{


}

LegStateBase::~LegStateBase() {

}

void LegStateBase::setIsNow(bool isNow) {
  isNow_= isNow;
}

bool LegStateBase::isNow() const {
  return isNow_;
}

} /* namespace loco */
