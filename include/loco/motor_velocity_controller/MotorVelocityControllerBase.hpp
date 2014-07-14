/*
 * MotorVelocityControllerBase.hpp
 *
 *  Created on: Jul 14, 2014
 *      Author: labstudent
 */

#ifndef MOTORVELOCITYCONTROLLERBASE_HPP_
#define MOTORVELOCITYCONTROLLERBASE_HPP_

#include "tinyxml.h"

namespace loco {

class MotorVelocityControllerBase {
 public:
  MotorVelocityControllerBase();
  virtual ~MotorVelocityControllerBase();
  virtual bool initialize(double dt) = 0;
  virtual void advance(double dt) = 0;
  virtual bool loadParameters(const TiXmlHandle& handle) = 0;
};
}

#endif /* MOTORVELOCITYCONTROLLERBASE_HPP_ */
