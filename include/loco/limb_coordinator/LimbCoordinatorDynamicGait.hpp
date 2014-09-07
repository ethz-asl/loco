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
  LimbCoordinatorDynamicGait(LegGroup* legs, TorsoBase* torso, GaitPatternBase* gaitPattern, bool isUpdatingStridePhase=true);
  virtual ~LimbCoordinatorDynamicGait();

  void setIsUpdatingStridePhase(bool isUpdatingStridePhase);
  bool isUpdatingStridePhase() const;

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
  virtual bool advance(double dt);

  virtual GaitPatternBase* getGaitPattern();
  const GaitPatternBase& getGaitPattern() const;

  virtual bool loadParameters(const TiXmlHandle& handle);


  /*! Computes an interpolated version of the two controllers passed in as parameters.
   *  If t is 0, the current setting is set to limbCoordinator1, 1 -> limbCoordinator2, and values in between
   *  correspond to interpolated parameter set.
   * @param limbCoordinator1
   * @param limbCoordinator2
   * @param t interpolation parameter
   * @returns true if successful
   */
  virtual bool setToInterpolated(const LimbCoordinatorBase& limbCoordinator1, const LimbCoordinatorBase& limbCoordinator2, double t);

  /*! Overrides stride phase from gait pattern.
   * @param stridePhase cycle phase in [0, 1]
   */
  void setStridePhase(double stridePhase);
 private:
  bool isUpdatingStridePhase_;
  LegGroup* legs_;
  TorsoBase* torso_;
  GaitPatternBase* gaitPattern_;
  bool isLegGrounded_[4];
  bool shouldBeLegGrounded_[4];

  virtual void setShouldBeLegGrounded(int iLeg, bool shouldBeLegGrounded);

};

} /* namespace loco */

#endif /* LOCO_LIMBCOORDINATORDYNAMICGAIT_HPP_ */
