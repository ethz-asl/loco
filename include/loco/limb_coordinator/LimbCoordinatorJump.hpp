/*
 * LimbCoordinatorJump.hpp
 *
 *  Created on: Oct 6, 2014
 *      Author: gech
 */

#ifndef LOCO_LIMBCOORDINATORJUMP_HPP_
#define LOCO_LIMBCOORDINATORJUMP_HPP_

#include "loco/limb_coordinator/LimbCoordinatorBase.hpp"
#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"

namespace loco {

class LimbCoordinatorJump: public LimbCoordinatorBase
{
 public:
  LimbCoordinatorJump(LegGroup* legs, TorsoBase* torso);
  virtual ~LimbCoordinatorJump();

  virtual bool initialize(double dt);

  /*! Advance in time
  * @param dt  time step [s]
  */
  virtual bool advance(double dt);
  virtual bool loadParameters(const TiXmlHandle& handle);
  virtual GaitPatternBase* getGaitPattern();
 private:
   LegGroup* legs_;
   TorsoBase* torso_;
};

} /* namespace loco */

#endif /* LIMBCOORDINATORJUMP_HPP_ */
