/*!
* @file     GaitPatternBase.hpp
* @author   Christian Gehring
* @date     Jun, 2013
* @version  1.0
* @ingroup
* @brief
*/
#ifndef LOCO_GAITPATTERNBASE_HPP_
#define LOCO_GAITPATTERNBASE_HPP_

#include "tinyxml.h"

namespace loco {

class GaitPatternBase {
public:
  GaitPatternBase();

  virtual ~GaitPatternBase();

  /*! @returns the stride duration in seconds
   */
  virtual double getStrideDuration() = 0;

  /*! Sets the stride duration
   * @param strideDuration  stride duration in seconds
   */
  virtual void setStrideDuration(double strideDuration) = 0;

  /*! @returns the relative phase for the leg whose index is passed in. The number
    returned is always going to be between 0 and 1 (0 meaning it should still be in stance mode,
    1 - it is a stance leg again, anything in between means that it is a swing leg).
    The stridePhase is expected to be between 0 and 1.
  */
  virtual double getSwingPhaseForLeg(int iLeg) = 0;

  /*! @returns the relative phase for the leg whose index is passed in. The number
    returned is always going to be between 0 and 1 (0 meaning it should still be in stance mode,
    1 - it is a stance leg again, anything in between means that it is a swing leg).
    The stridePhase is expected to be between 0 and 1.
  */
  virtual double getSwingPhaseForLeg(int iLeg, double stridePhase) const = 0;

  //! returns the relative stance phase for the leg. If the leg is in swing mode, it returns -1
  virtual double getStancePhaseForLeg(int iLeg) = 0;

  /*!  @returns the relative stance phase for the leg. If the limb is in swing mode, it returns -1
  */
  virtual double getStancePhaseForLeg(int iLeg, double stridePhase) const = 0;

  //! @returns the total length of the swing phase in seconds for a a given stride duration
  virtual double getSwingDuration(int iLeg, double strideDuration) const = 0;

  //! @returns the total length of the stance phase in seconds for a a given stride duration
  virtual double getStanceDuration(int iLeg, double strideDuration) const = 0;

  //! @returns the total length of the stance phase in seconds
  virtual double getStanceDuration(int iLeg) = 0;

  /*!  @returns number of gait cycles
   */
  virtual unsigned long int getNGaitCycles() = 0;

  /*! @returns number of legs that are in stance mode
   * @param stridePhase   stride phase
   */
  virtual int getNumberOfStanceLegs(double stridePhase) = 0;

  /*! @returns the time left in stance in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeLeftInStance(int iLeg, double strideDuration, double stridePhase) const = 0;

  /*! @returns the time left in swing in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeLeftInSwing(int iLeg, double strideDuration, double stridePhase) const = 0;

  /*! @returns the time spent in stance in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeSpentInStance(int iLeg, double strideDuration, double stridePhase) const = 0;

  /*! @returns the time spent in swing in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeSpentInSwing(int iLeg, double strideDuration, double stridePhase) const = 0;

  /*! @returns time until next stance phase starts in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the stride
   */
  virtual double getTimeUntilNextStancePhase(int iLeg, double strideDuration, double stridePhase) const = 0;

  /*! @returns time until next swing phase starts in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the stride
   */
  virtual double getTimeUntilNextSwingPhase(int iLeg, double strideDuration, double stridePhase) const = 0;


  virtual bool initialize(double dt) = 0;

  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual bool advance(double dt) = 0;



  virtual bool shouldBeLegGrounded(int iLeg) = 0;

  virtual double getStridePhase() = 0;

  virtual bool loadParameters(const TiXmlHandle& handle) = 0;

  const std::string& getName() const;
  void setName( const std::string& name);

  /*!
    computed an interpolated version of the two gaits passed in as parameters.
    if t is 0, the current gait is set to gait1, 1 -> gait 2, and values in between
    correspond to interpolated gaits.
  */
  virtual bool setToInterpolated(const GaitPatternBase& gaitPattern1, const GaitPatternBase& gaitPattern2, double t);

protected:
  std::string name_;
};

} // namespace loco


#endif /* LOCO_GAITPATTERNBASE_HPP_ */
