/*!
* @file     LimbCoordinatorBase.hpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#ifndef LOCO_LIMBCOORDINATIONBASE_HPP_
#define LOCO_LIMBCOORDINATIONBASE_HPP_

#include "loco/gait_pattern/GaitPatternBase.hpp"
#include "tinyxml.h"


namespace loco {

class LimbCoordinatorBase {
 public:
  LimbCoordinatorBase();
  virtual ~LimbCoordinatorBase();

  virtual bool initialize(double dt) = 0;
  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual bool advance(double dt) = 0;

  virtual GaitPatternBase* getGaitPattern() = 0;


  virtual bool loadParameters(const TiXmlHandle& handle) = 0;

  /*! Computes an interpolated version of the two controllers passed in as parameters.
   *  If t is 0, the current setting is set to limbCoordinator1, 1 -> limbCoordinator2, and values in between
   *  correspond to interpolated parameter set.
   * @param limbCoordinator1
   * @param limbCoordinator2
   * @param t interpolation parameter
   * @returns true if successful
   */
  virtual bool setToInterpolated(const LimbCoordinatorBase& limbCoordinator1, const LimbCoordinatorBase& limbCoordinator2, double t);
};

} /* namespace loco */

#endif /* LOCO_LIMBCOORDINATIONBASE_HPP_ */
