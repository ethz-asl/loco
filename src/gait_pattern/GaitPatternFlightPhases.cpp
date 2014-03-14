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

double loco::GaitPatternFlightPhases::getSwingPhaseForLeg(int iLeg) {
  double absolutePhase = cyclePhase;
  //the absolute phase parameter should be in the range [0-1], so make it so just in case...
  while (absolutePhase < 0) absolutePhase += 1; while (absolutePhase > 1) absolutePhase -= 1;

  return getRelativePhaseFromAbsolutePhaseInRange(absolutePhase, footFallPatterns[iLeg].footLiftOff, footFallPatterns[iLeg].footStrike);
}

double loco::GaitPatternFlightPhases::getStancePhaseForLeg(int iLeg) {

  const double absolutePhase = cyclePhase;

  double timeUntilFootLiftOff = footFallPatterns[iLeg].getPhaseLeftUntilFootLiftOff(absolutePhase);
  double timeUntilFootStrike = footFallPatterns[iLeg].getPhaseLeftUntilFootStrike(absolutePhase);

  //see if we're in swing mode...
  if (timeUntilFootStrike < timeUntilFootLiftOff)
    return 0;

  double swingPhaseRange = footFallPatterns[iLeg].footStrike - footFallPatterns[iLeg].footLiftOff;
  double timeSinceFootStrike = 1 - timeUntilFootLiftOff - swingPhaseRange;

  return timeSinceFootStrike / (timeSinceFootStrike + timeUntilFootLiftOff);
}

double loco::GaitPatternFlightPhases::getStanceDuration(int iLeg) {


  double swingDuration = footFallPatterns[iLeg].footStrike - footFallPatterns[iLeg].footLiftOff;

  return 1-swingDuration;
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

void loco::GaitPatternFlightPhases::advance(double dt) {
  assert(strideDuration!=0.0);
  cyclePhase += dt/strideDuration;
  if (cyclePhase > 1.0) {
    cyclePhase = 0.0;
  }

}

bool loco::GaitPatternFlightPhases::shouldBeLegGrounded(int iLeg) {
  return getSwingPhaseForLeg(iLeg)==-1;
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


inline double GaitPatternFlightPhases::getRelativePhaseFromAbsolutePhaseInRange(double phase, double start, double end){
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
  if (start == end) return -1; // added (Christian)
  double result = (phase - start) / (end - start);

  if (result < 0 || result > 1)
    result = -1;

  return result;
}

double GaitPatternFlightPhases::getFootLiftOffPhase(int iLeg) {
   return footFallPatterns[iLeg].footLiftOff;
}
double GaitPatternFlightPhases::getFootTouchDownPhase(int iLeg) {
  return footFallPatterns[iLeg].footStrike;
}

} /* namespace loco */
