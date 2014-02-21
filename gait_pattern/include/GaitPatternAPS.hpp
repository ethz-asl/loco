/*
 * GaitPatternAPS.hpp
 *
 *  Created on: Feb 21, 2014
 *      Author: gech
 */

#ifndef LOCO_GAITPATTERNAPS_HPP_
#define LOCO_GAITPATTERNAPS_HPP_

#include "GaitPatternAPS.hpp"

namespace loco {


class GaitPatternAPS: public GaitAPS, public GaitPatternBase {
  friend class ParameterManager;
public:
  GaitPatternAPS();
  virtual ~GaitPatternAPS();


  /*! Loads the parameters from the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  virtual bool loadParametersFromXML(TiXmlHandle &hParameterSet, DynamicArray<GenericLeg*>& legs, double dt);

  /*! Stores the current parameters in the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  virtual bool saveParametersInXML(TiXmlHandle &hParameterSet);

  /**
    computed an interpolated version of the two gaits passed in as parameters.
    if t is 0, the current gait is set to gait1, 1 -> gait 2, and values in between
    correspond to interpolated gaits.
  */
  virtual void setToInterpolatedGait(const GaitPatternAPS& gait1, const GaitPatternAPS& gait2, double t);


  /*! Gets the index of the footfall pattern in array footFallPatterns for a leg
   * @param leg   reference to the leg
   * @return index of the footfall pattern, if leg is not found, it returns -1
   */
  virtual int getLegIndex(GenericLeg* leg);

  /**
    returns the relative phase for the leg whose index is passed in. The number
    returned is always going to be between 0 and 1 (0 meaning it should still be in stance mode,
    1 - it is a stance leg again, anything in between means that it is a swing leg).
    The absoultePhase is expected to be between 0 and 1.
  */
  virtual double getRelativePhaseForLeg(GenericLeg* leg, double absolutePhase);

  //! returns the relative stance phase for the leg. If the leg is in swing mode, it returns -1
  virtual double getStancePhaseForLeg(GenericLeg* leg, double absolutePhase);

  //! returns the relative stance phase for the leg. If the leg is in swing mode, it returns -1
  virtual double getStancePhaseForLeg(int iLeg, double absolutePhase);

  //! returns the total length (in unitless phase measurement) of the stance phase
  virtual double getStanceDuration(GenericLeg* leg);


  virtual double getStrideDuration();
  virtual void setStrideDuration(double strideDuration);

  virtual unsigned long int getNumGaitCycles();

  std::string getLegName(int iLeg);


  void setVelocity(double value);

  double getVelocity();

protected:
  double velocity_;


};

} // namespace loco



#endif /* LOCO_GAITPATTERNAPS_HPP_ */
