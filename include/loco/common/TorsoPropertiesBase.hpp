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
  virtual const Position& getCenterOfMassInBaseFrame() const;
  virtual void setCenterOfMassInBaseFrame(const Position& centerOfMassInBaseFrame);

 private:
  double mass_;
  Position centerOfMassInBaseFrame_;
};

} /* namespace loco */

#endif /* LOCO_TORSOPROPERTIESBASE_HPP_ */
