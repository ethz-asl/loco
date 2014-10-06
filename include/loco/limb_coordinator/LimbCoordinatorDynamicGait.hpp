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
	int state_[4];

  LimbCoordinatorDynamicGait(LegGroup* legs, TorsoBase* torso, GaitPatternBase* gaitPattern, bool isUpdatingStridePhase=true);
  virtual ~LimbCoordinatorDynamicGait();

  void setIsUpdatingStridePhase(bool isUpdatingStridePhase);
  bool isUpdatingStridePhase() const;


  /**
    returns true if the leg is in stance mode, false otherwise.
  */
  virtual bool isLegInStanceMode(int iLeg);





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
};

} /* namespace loco */

#endif /* LOCO_LIMBCOORDINATORDYNAMICGAIT_HPP_ */
