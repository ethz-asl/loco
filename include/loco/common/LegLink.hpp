/*
 * LegLink.hpp
 *
 *  Created on: May 18, 2014
 *      Author: gech
 */

#ifndef LOCO_LEGLINK_HPP_
#define LOCO_LEGLINK_HPP_

#include "loco/common/TypeDefs.hpp"


#include "loco/common/LegBase.hpp"



namespace loco {

class LegBase;

class LegLink {
 public:
  LegLink();
  virtual ~LegLink();

  void setMass(double mass);
  double getMass() const;
  const LegBase::TranslationJacobian& getTranslationJacobianBaseToCoMInBaseFrame() const;
  void setTranslationJacobianBaseToCoMInBaseFrame(const LegBase::TranslationJacobian& jacobian);
  void setBaseToCoMPositionInBaseFrame(const Position& position);
  const Position& getBaseToCoMPositionInBaseFrame();
 protected:
  double mass_;
  Position positionBaseToCoMInBaseFrame_;
  LegBase::TranslationJacobian jacobianBaseToCoMInBaseFrame_;
};

} /* namespace loco */

#endif /* LOCO_LEGLINK_HPP_ */
