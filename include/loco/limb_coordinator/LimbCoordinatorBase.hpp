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

  virtual bool isLegGrounded(int iLeg) = 0;
  virtual bool shouldBeLegGrounded(int iLeg) = 0;
  virtual bool isAndShouldBeLegGrounded(int iLeg) = 0;
  /**
    returns true if the leg is in stance mode, false otherwise.
  */
  virtual bool isLegInStanceMode(int iLeg) = 0;

  /**
    returns true if the leg is in swing mode, false otherwise.
  */
  virtual bool isLegInSwingMode(int iLeg) = 0;

  virtual bool initialize(double dt) = 0;
  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual bool advance(double dt) = 0;

  virtual void setIsLegGrounded(int iLeg, bool isLegGrounded) = 0;

  virtual GaitPatternBase* getGaitPattern() = 0;


  virtual bool loadParameters(const TiXmlHandle& handle) = 0;
};

} /* namespace loco */

#endif /* LOCO_LIMBCOORDINATIONBASE_HPP_ */
