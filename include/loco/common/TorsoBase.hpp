/*
 * StateBase.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_STATEBASE_HPP_
#define LOCO_STATEBASE_HPP_


#include "loco/common/TypeDefs.hpp"

#include "loco/common/TorsoStateDesired.hpp"
#include "loco/common/TorsoStateMeasured.hpp"
#include "loco/common/TorsoPropertiesBase.hpp"
namespace loco {

//! Base class for a torso
/*! This should be used only as a data container
 *
 */
class TorsoBase {
 public:
  TorsoBase();
  virtual ~TorsoBase();

  virtual TorsoStateMeasured& getMeasuredState() = 0;
  virtual TorsoStateDesired& getDesiredState() = 0;
  virtual const TorsoStateDesired& getDesiredState() const = 0;
  virtual TorsoPropertiesBase& getProperties() = 0;

  virtual double getStridePhase() = 0;
  virtual void setStridePhase(double stridePhase) = 0;

  virtual bool initialize(double dt) = 0;

  /*! Advances in time, i.e. updates states
   * @param dt time step between updates
   */
  virtual bool advance(double dt) = 0;

  virtual void setDesiredBaseTwistInHeadingFrame(const Twist& desiredBaseTwistInHeadingFrame) = 0;

  friend std::ostream& operator << (std::ostream& out, const TorsoBase& torso);

};

} /* namespace loco */

#endif /* STATEBASE_HPP_ */
