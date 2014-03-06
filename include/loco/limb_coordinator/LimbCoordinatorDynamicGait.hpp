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
#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"

namespace loco {

class LimbCoordinatorDynamicGait: public LimbCoordinatorBase {
 public:
  LimbCoordinatorDynamicGait(LegGroup* legs, TorsoBase* torso, GaitPatternBase* gaitPattern);
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


 virtual bool initialize(double dt);

  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual void advance(double dt);

  virtual GaitPatternBase* getGaitPattern();

  virtual bool loadParameters(const TiXmlHandle& handle);

 private:
  LegGroup* legs_;
  TorsoBase* torso_;
  GaitPatternBase* gaitPattern_;
  bool isLegGrounded_[4];
  bool shouldBeLegGrounded_[4];

  virtual void setShouldBeLegGrounded(int iLeg, bool shouldBeLegGrounded);

};

} /* namespace loco */

#endif /* LOCO_LIMBCOORDINATORDYNAMICGAIT_HPP_ */
