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
      positionBaseToCenterOfMassInBaseFrame_()
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

const Position& TorsoPropertiesBase::getBaseToCenterOfMassPositionInBaseFrame() const
{
  return positionBaseToCenterOfMassInBaseFrame_;
}

void TorsoPropertiesBase::setBaseToCenterOfMassPositionInBaseFrame(const Position& centerOfMassInBaseFrame)
{
  positionBaseToCenterOfMassInBaseFrame_ = centerOfMassInBaseFrame;
}

const LinearAcceleration& TorsoPropertiesBase::getGravity() const
{
  return gravity_;
}

void TorsoPropertiesBase::setGravity(const LinearAcceleration& gravity)
{
  gravity_ = gravity;
  gravityAxisInWorldFrame_ = Vector(gravity).normalized();
}

const Vector& TorsoPropertiesBase::getHeadingAxisInBaseFrame()
{
  return headingAxisInBaseFrame_;
}

const Vector& TorsoPropertiesBase::getLateralAxisInBaseFrame()
{
  return lateralAxis_;
}

const Vector& TorsoPropertiesBase::getVerticalAxisInBaseFrame()
{
  return verticalAxisInBaseFrame_;
}

void TorsoPropertiesBase::setHeadingAxisInBaseFrame(const Vector& axis)
{
  headingAxisInBaseFrame_ = axis;
}

void TorsoPropertiesBase::setLateralAxisInBaseFrame(const Vector& axis)
{
  lateralAxis_ = axis;
}

void TorsoPropertiesBase::setVerticalAxisInBaseFrame(const Vector& axis)
{
  verticalAxisInBaseFrame_ = axis;
}

const Vector& TorsoPropertiesBase::getGravityAxisInWorldFrame() {
  return gravityAxisInWorldFrame_;
}




} /* namespace loco */
