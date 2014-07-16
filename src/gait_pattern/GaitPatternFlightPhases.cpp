/*
 * GaitPatternFlightPhases.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: gech
 */

#include "loco/gait_pattern/GaitPatternFlightPhases.hpp"

namespace loco {

GaitPatternFlightPhases::GaitPatternFlightPhases():
    isInitialized_(false)
{


}

GaitPatternFlightPhases::~GaitPatternFlightPhases() {

}



double loco::GaitPatternFlightPhases::getStrideDuration() {
  return strideDuration;
}

void loco::GaitPatternFlightPhases::setStrideDuration(double strideDuration) {
  this->strideDuration = strideDuration;
}

double GaitPatternFlightPhases::getSwingPhaseForLeg(int iLeg) {
  return getSwingPhaseForLeg(iLeg,  cyclePhase);
}

double GaitPatternFlightPhases::getSwingPhaseForLeg(int iLeg, double stridePhase) const {
  //the absolute phase parameter should be in the range [0-1], so make it so just in case...
  while (stridePhase < 0) stridePhase += 1; while (stridePhase > 1) stridePhase -= 1;

  return getRelativePhaseFromAbsolutePhaseInRange(stridePhase, footFallPatterns[iLeg].liftOffPhase, footFallPatterns[iLeg].strikePhase);
}

double loco::GaitPatternFlightPhases::getStancePhaseForLeg(int iLeg, double stridePhase) const {

  double timeUntilFootLiftOff = footFallPatterns[iLeg].getPhaseLeftUntilFootLiftOff(stridePhase);
  double timeUntilFootStrike = footFallPatterns[iLeg].getPhaseLeftUntilFootStrike(stridePhase);

  if (footFallPatterns[iLeg].strikePhase == footFallPatterns[iLeg].liftOffPhase) {
    return 1.0; // added (Christian)
  }

  //see if we're in swing mode...
  if (timeUntilFootStrike < timeUntilFootLiftOff) {
//    return 0.0>;
    return -1; // changed with new gait pattern from SC
  }


  double swingPhaseRange = footFallPatterns[iLeg].strikePhase - footFallPatterns[iLeg].liftOffPhase;
  double timeSinceFootStrike = 1.0 - timeUntilFootLiftOff - swingPhaseRange;

  return timeSinceFootStrike / (timeSinceFootStrike + timeUntilFootLiftOff);
}

double GaitPatternFlightPhases::getStancePhaseForLeg(int iLeg) {
  return getStancePhaseForLeg(iLeg, cyclePhase);
}




double GaitPatternFlightPhases::getStanceDuration(int iLeg) {
  return getStanceDuration(iLeg, strideDuration);
}

double GaitPatternFlightPhases::getSwingDuration(int iLeg, double strideDuration) const {
  return strideDuration - getStanceDuration(iLeg, strideDuration);
}

double GaitPatternFlightPhases::getStanceDuration(int iLeg, double strideDuration) const {
  return (1.0-(footFallPatterns[iLeg].strikePhase - footFallPatterns[iLeg].liftOffPhase)) * strideDuration;
}

unsigned long int loco::GaitPatternFlightPhases::getNGaitCycles() {
  return numGaitCycles;
}

bool loco::GaitPatternFlightPhases::initialize(double dt) {
  numGaitCycles = 0;
  cyclePhase = initCyclePhase;
  isInitialized_ = true;
  return true;
}

bool loco::GaitPatternFlightPhases::isInitialized()
{
  return isInitialized_;
}

bool loco::GaitPatternFlightPhases::advance(double dt) {
  if (strideDuration == 0.0) {
    cyclePhase = 0.0;
    return true;
  }
  cyclePhase += dt/strideDuration;
  if (cyclePhase > 1.0) {
    cyclePhase = 0.0;
    numGaitCycles++;
  }
  return true;

}

bool loco::GaitPatternFlightPhases::shouldBeLegGrounded(int iLeg) {
//  printf("leg %d: stance phase: %f\n",iLeg, getStancePhaseForLeg(iLeg));
//  return (getStancePhaseForLeg(iLeg)!=0.0);
  return (getStancePhaseForLeg(iLeg) >= 0.0); // changed since returns now -1 for swing mode
//  return (getStancePhaseForLeg(iLeg) !=0.0 || footFallPatterns[iLeg].liftOffPhase == footFallPatterns[iLeg].strikePhase); // added (Christian)
}

double loco::GaitPatternFlightPhases::getStridePhase() {
  return cyclePhase;
}

bool loco::GaitPatternFlightPhases::loadParameters(const TiXmlHandle& handle) {
  double value, liftOff, touchDown;
    TiXmlElement* pElem;

    /* gait pattern */
    TiXmlHandle hFootFallPattern(handle.FirstChild("GaitPattern").FirstChild("FlightPhases"));
    pElem = hFootFallPattern.Element();
    if (!pElem) {
      printf("Could not find FlightPhases\n");
      return false;
    }
    if (pElem->QueryDoubleAttribute("cycleDuration", &this->strideDuration)!=TIXML_SUCCESS) {
      printf("Could not find FlightPhases:cycleDuration\n");
      return false;
    }

    if (pElem->QueryDoubleAttribute("initCyclePhase", &this->initCyclePhase)!=TIXML_SUCCESS) {
      printf("Could not find FlightPhases:initCyclePhase\n");
      return false;
    }

    /* foot fall pattern */


    this->footFallPatterns.clear();

    pElem = hFootFallPattern.FirstChild("LF").Element();
    if(!pElem) {
      printf("Could not find FlightPhases:LF\n");
      return false;
    }

    if (pElem->QueryDoubleAttribute("liftOff", &liftOff)!=TIXML_SUCCESS) {
      printf("Could not find LF:liftOff\n");
      return false;
    }

    if (pElem->QueryDoubleAttribute("touchDown", &touchDown)!=TIXML_SUCCESS) {
      printf("Could not find LF:touchDown\n");
      return false;
    }
    footFallPatterns.push_back(FootFallPattern(liftOff, touchDown));


    pElem = hFootFallPattern.FirstChild("RF").Element();
    if(!pElem) {
      printf("Could not find RF\n");
      return false;
    }

    if (pElem->QueryDoubleAttribute("liftOff", &liftOff)!=TIXML_SUCCESS) {
      printf("Could not find RF:liftOff\n");
      return false;
    }
    if (pElem->QueryDoubleAttribute("touchDown", &touchDown)!=TIXML_SUCCESS) {
      printf("Could not find GaitPattern:FootFallPattern:RF:touchDown\n");
      return false;
    }
    footFallPatterns.push_back(FootFallPattern(liftOff, touchDown));

    pElem = hFootFallPattern.FirstChild("LH").Element();
    if(!pElem) {
      printf("Could not find LH\n");
      return false;
    }
    if (pElem->QueryDoubleAttribute("liftOff", &liftOff)!=TIXML_SUCCESS) {
      printf("Could not find LH:liftOff\n");
      return false;
    }
    if (pElem->QueryDoubleAttribute("touchDown", &touchDown)!=TIXML_SUCCESS) {
      printf("Could not find LH:touchDown\n");
      return false;
    }
    footFallPatterns.push_back(FootFallPattern(liftOff, touchDown));

    pElem = hFootFallPattern.FirstChild("RH").Element();
    if(!pElem) {
      printf("Could not find RH\n");
      return false;
    }
    if (pElem->QueryDoubleAttribute("liftOff", &liftOff)!=TIXML_SUCCESS) {
      printf("Could not find RH:liftOff\n");
      return false;
    }
    if (pElem->QueryDoubleAttribute("touchDown", &touchDown)!=TIXML_SUCCESS) {
      printf("Could not find RH:touchDown\n");
      return false;
    }
    footFallPatterns.push_back(FootFallPattern(liftOff, touchDown));

    numGaitCycles = 0;
    return true;
}


inline double GaitPatternFlightPhases::getRelativePhaseFromAbsolutePhaseInRange(double phase, double start, double end) const {
  if (start < 0){
    end -= start;
    phase -= start;
    start -= start;
    if (phase > 1) phase -= 1;
  }
  if (end > 1){
    start -= (end-1);
    phase -= (end-1);
    end -= (end-1);
    if (phase < 0) phase += 1;
  }
//  printf("start: %lf, end: %lf\n", start, end);
  if (start == end) return -1; // added (Christian)
  double result = (phase - start) / (end - start);

  if (result < 0 || result > 1)
    result = -1;

  return result;
}

double GaitPatternFlightPhases::getFootLiftOffPhase(int iLeg) {
   return footFallPatterns[iLeg].liftOffPhase;
}
double GaitPatternFlightPhases::getFootTouchDownPhase(int iLeg) {
  return footFallPatterns[iLeg].strikePhase;
}


double GaitPatternFlightPhases::getTimeLeftInStance(int iLeg, double strideDuration, double stridePhase) const {
  double stancePhase = getStancePhaseForLeg(iLeg, stridePhase);
  if (stancePhase < 0 || stancePhase > 1) return 0;
  return getStanceDuration(iLeg, strideDuration) * (1-stancePhase);
}

double GaitPatternFlightPhases::getTimeLeftInSwing(int iLeg, double strideDuration, double stridePhase) const {
  double swingPhase = getSwingPhaseForLeg(iLeg, stridePhase);
  if (swingPhase < 0 || swingPhase > 1) return 0;
  return getSwingDuration(iLeg, strideDuration) * (1-swingPhase);
}

double GaitPatternFlightPhases::getTimeSpentInStance(int iLeg, double strideDuration, double stridePhase) const {
  double stancePhase = getStancePhaseForLeg(iLeg, stridePhase);
  if (stancePhase < 0 || stancePhase > 1) return 0;
  return getStanceDuration(iLeg, strideDuration) * stancePhase;
}

double GaitPatternFlightPhases::getTimeSpentInSwing(int iLeg, double strideDuration, double stridePhase) const {
  double swingPhase = getSwingPhaseForLeg(iLeg, stridePhase);
  if (swingPhase < 0 || swingPhase > 1) return 0;
  return getSwingDuration(iLeg, strideDuration) * swingPhase;
}

double GaitPatternFlightPhases::getTimeUntilNextStancePhase(int iLeg, double strideDuration, double stridePhase) const {
  double stancePhase = getStancePhaseForLeg(iLeg, stridePhase);
  //the limb is already in stance phase, so the time until the next stance phase starts is one whole stridePhase minus the amount of time it's already spent in stance mode
  if (stancePhase >= 0 && stancePhase <= 1)
    return strideDuration - getTimeSpentInStance(iLeg, strideDuration, stridePhase);
  //if the limb is in swing phase, then we have as much time until the swing phase ends
  return getTimeLeftInSwing(iLeg, strideDuration, stridePhase);
}

double GaitPatternFlightPhases::getTimeUntilNextSwingPhase(int iLeg, double strideDuration, double stridePhase) const {
  double swingPhase = getSwingPhaseForLeg(iLeg, stridePhase);
  //the limb is already in swing phase, so the time until the next swing phase starts is one whole stridePhase minus the amount of time it's already spent in swing mode
  if (swingPhase >= 0 && swingPhase <= 1)
    return strideDuration - getTimeSpentInSwing(iLeg, strideDuration, stridePhase);
  //if the limb is in stance phase, then we have as much time until the stance phase ends
  return getTimeLeftInStance(iLeg, strideDuration, stridePhase);
}

} /* namespace loco */
