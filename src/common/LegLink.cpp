/*
 * Link.cpp
 *
 *  Created on: May 18, 2014
 *      Author: gech
 */

#include "loco/common/LegLink.hpp"
#include "loco/common/LegBase.hpp"
namespace loco {

LegLink::LegLink():
    mass_(0.0)
{


}

LegLink::~LegLink() {

}

void LegLink::setMass(double mass)
{
  mass_ = mass;
}

double LegLink::getMass() const {
  return mass_;
}

const LegBase::TranslationJacobian& LegLink::getTranslationJacobianBaseToCoMInBaseFrame() const {
  return jacobianBaseToCoMInBaseFrame_;
}
void LegLink::setTranslationJacobianBaseToCoMInBaseFrame(const LegBase::TranslationJacobian& jacobian) {
  jacobianBaseToCoMInBaseFrame_ = jacobian;
}

void LegLink::setBaseToCoMPositionInBaseFrame(const Position& position) {
  positionBaseToCoMInBaseFrame_ = position;
}
const Position& LegLink::getBaseToCoMPositionInBaseFrame() {
  return positionBaseToCoMInBaseFrame_;
}

} /* namespace loco */
