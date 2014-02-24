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


namespace loco {

class GaitPatternBase {
public:
  GaitPatternBase() {

  }

  virtual ~GaitPatternBase() {

  }

  /*! @returns the stride duration in seconds
   */
  virtual double getStrideDuration() = 0;

  /*! Sets the stride duration
   * @param strideDuration  stride duration in seconds
   */
  virtual void setStrideDuration(double strideDuration) = 0;

  /**
    returns the relative phase for the leg whose index is passed in. The number
    returned is always going to be between 0 and 1 (0 meaning it should still be in stance mode,
    1 - it is a stance leg again, anything in between means that it is a swing leg).
    The stridePhase is expected to be between 0 and 1.
  */
  virtual double getSwingPhaseForLeg(int iLeg) = 0;

  //! returns the relative stance phase for the leg. If the leg is in swing mode, it returns -1
  virtual double getStancePhaseForLeg(int iLeg) = 0;

  //! returns the total length (in unitless phase measurement) of the stance phase
  virtual double getStanceDuration(int iLeg) = 0;

  /*!  @returns number of gait cycles
   */
  virtual unsigned long int getNGaitCycles() = 0;

  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual void advance(double dt) = 0;



  virtual bool shouldBeLegGrounded(int iLeg) = 0;



};

} // namespace loco


#endif /* LOCO_GAITPATTERNBASE_HPP_ */
