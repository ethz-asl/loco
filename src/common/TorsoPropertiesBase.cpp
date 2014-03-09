/*
 * TorsoPropertiesBase.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/common/TorsoPropertiesBase.hpp"

namespace loco {

TorsoPropertiesBase::TorsoPropertiesBase()
    : mass_(0.0),
      centerOfMassInBaseFrame_()
{

}

TorsoPropertiesBase::~TorsoPropertiesBase()
{

}

double TorsoPropertiesBase::getMass() const
{
  return mass_;
}

void TorsoPropertiesBase::setMass(double mass)
{
  mass_ = mass;
}

const Position& TorsoPropertiesBase::getCenterOfMassInBaseFrame() const
{
  return centerOfMassInBaseFrame_;
}

void TorsoPropertiesBase::setCenterOfMassInBaseFrame(const Position& centerOfMassInBaseFrame)
{
  centerOfMassInBaseFrame_ = centerOfMassInBaseFrame;
}

} /* namespace loco */
