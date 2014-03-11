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
      positionBaseToCenterOfMassInBaseFrame_()
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

const Position& LegPropertiesBase::getBaseToCenterOfMassPositionInBaseFrame() const
{
  return positionBaseToCenterOfMassInBaseFrame_;
}

void LegPropertiesBase::setBaseToCenterOfMassPositionInBaseFrame(const Position& centerOfMassInBaseFrame)
{
  positionBaseToCenterOfMassInBaseFrame_ = centerOfMassInBaseFrame;
}

const Position& LegPropertiesBase::getDesiredDefaultSteppingPositionHipToFootInBaseFrame() const
{
  return desiredDefaultSteppingPositionHipToFootInBaseFrame_;
}
void LegPropertiesBase::setDesiredDefaultSteppingPositionHipToFootInBaseFrame(const Position& position)
{
  desiredDefaultSteppingPositionHipToFootInBaseFrame_ = position;
}

} /* namespace loco */
