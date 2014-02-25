/*!
* @file     LimbCoordinatorDynamicGait.hpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#ifndef LOCO_LIMBCOORDINATORDYNAMICGAIT_HPP_
#define LOCO_LIMBCOORDINATORDYNAMICGAIT_HPP_

#include "loco/limb_coordinator/LimbCoordinatorBase.hpp"
#include "loco/gait_pattern/GaitPatternBase.hpp"

namespace loco {

class LimbCoordinatorDynamicGait: public LimbCoordinatorBase {
 public:
  LimbCoordinatorDynamicGait(GaitPatternBase* gaitPattern);
  virtual ~LimbCoordinatorDynamicGait();

  virtual bool isLegGrounded(int iLeg);
  virtual bool shouldBeLegGrounded(int iLeg);
  virtual bool isAndShouldBeLegGrounded(int iLeg);
  /**
    returns true if the leg is in stance mode, false otherwise.
  */
  virtual bool isLegInStanceMode(int iLeg);

  /**
    returns true if the leg is in swing mode, false otherwise.
  */
  virtual bool isLegInSwingMode(int iLeg);

  virtual void setIsLegGrounded(int iLeg, bool isLegGrounded);

  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual void advance(double dt);

  virtual GaitPatternBase* getGaitPattern();

 private:
  GaitPatternBase* gaitPattern_;
  bool isLegGrounded_[4];
  bool shouldBeLegGrounded_[4];

  virtual void setShouldBeLegGrounded(int iLeg, bool shouldBeLegGrounded);

};

} /* namespace loco */

#endif /* LOCO_LIMBCOORDINATORDYNAMICGAIT_HPP_ */
