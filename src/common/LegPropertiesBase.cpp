/*
 * LegPropertiesBase.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/common/LegPropertiesBase.hpp"

namespace loco {

LegPropertiesBase::LegPropertiesBase()
    : mass_(0.0),
      centerOfMassInBaseFrame_()
{

}

LegPropertiesBase::~LegPropertiesBase()
{

}

double LegPropertiesBase::getMass() const
{
  return mass_;
}

void LegPropertiesBase::setMass(double mass)
{
  mass_ = mass;
}

const Position& LegPropertiesBase::getCenterOfMassInBaseFrame() const
{
  return centerOfMassInBaseFrame_;
}

void LegPropertiesBase::setCenterOfMassInBaseFrame(const Position& centerOfMassInBaseFrame)
{
  centerOfMassInBaseFrame_ = centerOfMassInBaseFrame;
}

} /* namespace loco */
