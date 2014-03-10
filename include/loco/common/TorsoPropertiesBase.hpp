/*
 * TorsoPropertiesBase.hpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef LOCO_TORSOPROPERTIESBASE_HPP_
#define LOCO_TORSOPROPERTIESBASE_HPP_

#include "TypeDefs.hpp"

namespace loco {

class TorsoPropertiesBase {
 public:
  TorsoPropertiesBase();
  virtual ~TorsoPropertiesBase();
  virtual bool update() = 0;
  virtual double getMass() const;
  virtual void setMass(double mass);
  virtual const Position& getBaseToCenterOfMassPositionInBaseFrame() const;
  virtual void setBaseToCenterOfMassPositionInBaseFrame(const Position& centerOfMassInBaseFrame);
  const LinearAcceleration& getGravity() const;
  void setGravity(const LinearAcceleration& gravity);

 private:
  LinearAcceleration gravity_;
  double mass_;
  Position positionBaseToCenterOfMassInBaseFrame_;
};

} /* namespace loco */

#endif /* LOCO_TORSOPROPERTIESBASE_HPP_ */
