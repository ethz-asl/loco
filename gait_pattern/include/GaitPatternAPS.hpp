/*
 * GaitPatternAPS.hpp
 *
 *  Created on: Feb 21, 2014
 *      Author: gech
 */

#ifndef LOCO_GAITPATTERNAPS_HPP_
#define LOCO_GAITPATTERNAPS_HPP_

#include "tinyxml.h"

#include "GaitPatternBase.hpp"
#include "GaitAPS.hpp"

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
  virtual bool loadParametersFromXML(TiXmlHandle &hParameterSet, double dt);

  /*! Stores the current parameters in the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  virtual bool saveParametersInXML(TiXmlHandle &hParameterSet);


  bool initialize(const APS& aps, double dt);

  /**
    returns the relative phase for the leg whose index is passed in. The number
    returned is always going to be between 0 and 1 (0 meaning it should still be in stance mode,
    1 - it is a stance leg again, anything in between means that it is a swing leg).
    The absoultePhase is expected to be between 0 and 1.
  */
  virtual double getSwingPhaseForLeg(int iLeg);

  //! returns the relative stance phase for the leg. If the leg is in swing mode, it returns -1
  virtual double getStancePhaseForLeg(int iLeg);


  //! returns the total length (in unitless phase measurement) of the stance phase
  virtual double getStanceDuration(int iLeg);


  virtual double getStrideDuration();
  virtual void setStrideDuration(double strideDuration);

  virtual unsigned long int getNGaitCycles();


  void setVelocity(double value);

  double getVelocity();

  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual void advance(double dt);

  virtual bool shouldBeLegGrounded(int iLeg);

protected:
  double velocity_;


};

} // namespace loco



#endif /* LOCO_GAITPATTERNAPS_HPP_ */
