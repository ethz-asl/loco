/*
 * LegPropertiesBase.hpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef LOCO_LEGPROPERTIESBASE_HPP_
#define LOCO_LEGPROPERTIESBASE_HPP_

#include "TypeDefs.hpp"

namespace loco {

class LegPropertiesBase
{
 public:
  LegPropertiesBase();
  virtual ~LegPropertiesBase();
  virtual bool update() = 0;
  virtual double getMass() const;
  virtual void setMass(double mass);
  virtual const Position& getBaseToCenterOfMassPositionInBaseFrame() const;
  virtual void setBaseToCenterOfMassPositionInBaseFrame(const Position& centerOfMassInBaseFrame);

 private:

  //! The total mass of the leg.
  double mass_;

  //! The center of the total mass of the leg.
  Position positionBaseToCenterOfMassInBaseFrame_;
};

} /* namespace loco */

#endif /* LOCO_LEGPROPERTIESBASE_HPP_ */
