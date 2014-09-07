/*
 * GaitPatternAPS.hpp
 *
 *  Created on: Feb 21, 2014
 *      Author: gech
 */

#ifndef LOCO_GAITPATTERNAPS_HPP_
#define LOCO_GAITPATTERNAPS_HPP_

#include "tinyxml.h"

#include "loco/gait_pattern/GaitPatternBase.hpp"
#include "loco/gait_pattern/GaitAPS.hpp"

#include <vector>

namespace loco {


class GaitPatternAPS: public GaitAPS, public GaitPatternBase {
public:
  GaitPatternAPS();
  virtual ~GaitPatternAPS();


  /*! Loads the parameters from the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  virtual bool loadParameters(const TiXmlHandle &hParameterSet);

  /*! Stores the current parameters in the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  virtual bool saveParameters(TiXmlHandle &hParameterSet);


  bool initialize(const APS& aps, double dt);

  virtual bool initialize(double dt);

  bool isInitialized();

  /*! @returns the relative phase for the leg whose index is passed in. The number
    returned is always going to be between 0 and 1 (0 meaning it should still be in stance mode,
    1 - it is a stance leg again, anything in between means that it is a swing leg).
    The stridePhase is expected to be between 0 and 1.
  */
  virtual double getSwingPhaseForLeg(int iLeg);

  /*! @returns the relative phase for the leg whose index is passed in. The number
    returned is always going to be between 0 and 1 (0 meaning it should still be in stance mode,
    1 - it is a stance leg again, anything in between means that it is a swing leg).
    The stridePhase is expected to be between 0 and 1.
  */
  virtual double getSwingPhaseForLeg(int iLeg, double stridePhase) const;

  //! returns the relative stance phase for the leg. If the leg is in swing mode, it returns -1
  virtual double getStancePhaseForLeg(int iLeg);

  /*!  @returns the relative stance phase for the leg. If the limb is in swing mode, it returns -1
  */
  virtual double getStancePhaseForLeg(int iLeg, double stridePhase) const;

  //! @returns the total length of the stance phase in seconds
  virtual double getStanceDuration(int iLeg);

  //! @returns the total length of the swing phase in seconds for a a given stride duration
  virtual double getSwingDuration(int iLeg, double strideDuration) const;

  //! @returns the total length of the stance phase in seconds for a a given stride duration
  virtual double getStanceDuration(int iLeg, double strideDuration) const;


  virtual double getStrideDuration();
  virtual void setStrideDuration(double strideDuration);

  virtual unsigned long int getNGaitCycles();


  void setVelocity(double value);

  double getVelocity();

  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual bool advance(double dt);

  virtual bool shouldBeLegGrounded(int iLeg);

  /*! @returns stride (cycle) phase, which is between [0, 1].
   */
  virtual double getStridePhase() const;

  /*! Sets the stride (cycle phase), which is between [0, 1].
   * @param stridePhase cycle phase
   */
  virtual void setStridePhase(double stridePhase);


  /*! @returns the time left in stance in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeLeftInStance(int iLeg, double strideDuration, double stridePhase) const;

  /*! @returns the time left in swing in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeLeftInSwing(int iLeg, double strideDuration, double stridePhase) const;

  /*! @returns the time spent in stance in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeSpentInStance(int iLeg, double strideDuration, double stridePhase) const;

  /*! @returns the time spent in swing in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeSpentInSwing(int iLeg, double strideDuration, double stridePhase) const;

  /*! @returns time until next stance phase starts in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the stride
   */
  virtual double getTimeUntilNextStancePhase(int iLeg, double strideDuration, double stridePhase) const;

  /*! @returns time until next swing phase starts in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the stride
   */
  virtual double getTimeUntilNextSwingPhase(int iLeg, double strideDuration, double stridePhase) const;


  /*! @returns number of legs that are in stance mode
   * @param stridePhase   stride phase
   */
  virtual int getNumberOfStanceLegs(double stridePhase);
protected:
  bool isInitialized_;
  double velocity_;
  APS initAPS_;


};

} // namespace loco



#endif /* LOCO_GAITPATTERNAPS_HPP_ */
