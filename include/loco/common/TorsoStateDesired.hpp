/*
 * TorsoStateDesired.hpp
 *
 *  Created on: Mar 4, 2014
 *      Author: gech
 */

#ifndef LOCO_TORSOSTATEDESIRED_HPP_
#define LOCO_TORSOSTATEDESIRED_HPP_

#include "loco/common/TorsoStateBase.hpp"

namespace loco {

//! Desired state of the torso
class TorsoStateDesired: public TorsoStateBase {
 public:
  TorsoStateDesired();
  virtual ~TorsoStateDesired();

  const RotationQuaternion&  getWorldToHeadingOrientation() const = delete;
  void setWorldToHeadingOrientation(const RotationQuaternion& orientation) = delete;

  const RotationQuaternion&  getHeadingToBaseOrientation() const = delete;
  void setHeadingToBaseOrientation(const RotationQuaternion& orientation) = delete;
};

} /* namespace loco */

#endif /* LOCO_TORSOSTATEDESIRED_HPP_ */
