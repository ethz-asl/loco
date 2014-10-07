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
      positionBaseToCenterOfMassInBaseFrame_(),
      desiredDefaultSteppingPositionHipToFootInControlFrame_(0.2, 0.3, 0.4)
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

const Position& LegPropertiesBase::getDesiredDefaultSteppingPositionHipToFootInControlFrame() const
{
  return desiredDefaultSteppingPositionHipToFootInControlFrame_;
}

Position& LegPropertiesBase::getDesiredDefaultSteppingPositionHipToFootInControlFrame() {
  return desiredDefaultSteppingPositionHipToFootInControlFrame_;
}


void LegPropertiesBase::setDesiredDefaultSteppingPositionHipToFootInControlFrame(const Position& position)
{
  desiredDefaultSteppingPositionHipToFootInControlFrame_ = position;
}

} /* namespace loco */
