/*
 * MissionControlBase.hpp
 *
 *  Created on: Mar 7, 2014
 *      Author: gech
 */

#ifndef LOCO_MISSIONCONTROLBASE_HPP_
#define LOCO_MISSIONCONTROLBASE_HPP_

#include "loco/common/TypeDefs.hpp"
#include "tinyxml.h"

namespace loco {

class MissionControlBase {
 public:
  MissionControlBase();
  virtual ~MissionControlBase();

  virtual const Twist& getDesiredBaseTwistInHeadingFrame() const = 0;
  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;
  virtual bool loadParameters(const TiXmlHandle& handle) = 0;

  /*! Computes an interpolated version of the two mission controllers passed in as parameters.
   *  If t is 0, the current setting is set to missionController1, 1 -> missionController2, and values in between
   *  correspond to interpolated parameter set.
   * @param missionController1
   * @param missionController2
   * @param t interpolation parameter
   * @returns true if successful
   */
  virtual bool setToInterpolated(const MissionControlBase& missionController1, const MissionControlBase& missionController2, double t);

};

} /* namespace loco */

#endif /* LOCO_MISSIONCONTROLBASE_HPP_ */
