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
  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;
  virtual double getMass() const;
  virtual void setMass(double mass);
  virtual const Position& getBaseToCenterOfMassPositionInBaseFrame() const;
  virtual void setBaseToCenterOfMassPositionInBaseFrame(const Position& centerOfMassInBaseFrame);

  virtual const Position& getDesiredDefaultSteppingPositionHipToFootInBaseFrame() const;
  virtual void setDesiredDefaultSteppingPositionHipToFootInBaseFrame(const Position& position);
  virtual double getLegLength() = 0;
 private:

  //! The total mass of the leg.
  double mass_;

  //! The center of the total mass of the leg.
  Position positionBaseToCenterOfMassInBaseFrame_;

  //! default stepping offset with respect to the hip
  Position desiredDefaultSteppingPositionHipToFootInBaseFrame_;
};

} /* namespace loco */

#endif /* LOCO_LEGPROPERTIESBASE_HPP_ */
