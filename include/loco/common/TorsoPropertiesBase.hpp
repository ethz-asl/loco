/*
 * TorsoPropertiesBase.hpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef LOCO_TORSOPROPERTIESBASE_HPP_
#define LOCO_TORSOPROPERTIESBASE_HPP_

#include "loco/common/TypeDefs.hpp"

namespace loco {

class TorsoPropertiesBase {
 public:
  TorsoPropertiesBase();
  virtual ~TorsoPropertiesBase();
  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;
  virtual double getMass() const;
  virtual void setMass(double mass);
  virtual const Position& getBaseToCenterOfMassPositionInBaseFrame() const;
  virtual void setBaseToCenterOfMassPositionInBaseFrame(const Position& centerOfMassInBaseFrame);
  const LinearAcceleration& getGravity() const;
  void setGravity(const LinearAcceleration& gravity);

  const Vector& getGravityAxisInWorldFrame();
  const Vector& getHeadingAxisInBaseFrame();
  const Vector& getLateralAxisInBaseFrame();
  const Vector& getVerticalAxisInBaseFrame();

  void setHeadingAxisInBaseFrame(const Vector& axis);
  void setLateralAxisInBaseFrame(const Vector& axis);
  void setVerticalAxisInBaseFrame(const Vector& axis);

 private:
  LinearAcceleration gravity_;
  Vector gravityAxisInWorldFrame_;
  double mass_;
  Position positionBaseToCenterOfMassInBaseFrame_;

  Vector headingAxisInBaseFrame_;
  Vector lateralAxis_;
  Vector verticalAxisInBaseFrame_;
};

} /* namespace loco */

#endif /* LOCO_TORSOPROPERTIESBASE_HPP_ */
