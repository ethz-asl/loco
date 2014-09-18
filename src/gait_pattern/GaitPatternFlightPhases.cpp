/*
 * GaitPatternFlightPhases.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: gech
 */

#include "loco/gait_pattern/GaitPatternFlightPhases.hpp"
#include "loco/temp_helpers/math.hpp"

#include <stdexcept>

namespace loco {

GaitPatternFlightPhases::GaitPatternFlightPhases():
    isInitialized_(false),
    initCyclePhase_(0.0),
    cyclePhase_(0.0),
    numGaitCycles_(0),
    strideDuration_(0.0)
{


}

GaitPatternFlightPhases::~GaitPatternFlightPhases() {

}



double loco::GaitPatternFlightPhases::getStrideDuration() {
  return strideDuration_;
}

void loco::GaitPatternFlightPhases::setStrideDuration(double strideDuration) {
  this->strideDuration_ = strideDuration;
}

double GaitPatternFlightPhases::getSwingPhaseForLeg(int iLeg) {
  return getSwingPhaseForLeg(iLeg,  cyclePhase_);
}

double GaitPatternFlightPhases::getSwingPhaseForLeg(int iLeg, double stridePhase) const {
  int pIndex = getStepPatternIndexForLeg(iLeg);
  //by default all limbs are in stance mode
  if (pIndex == -1)
    return -1;

  if (stepPatterns_[pIndex].liftOffPhase == stepPatterns_[pIndex].strikePhase) {
    return -1; // added by Christian
  }

  double timeUntilFootLiftOff = stepPatterns_[pIndex].getPhaseLeftUntilLiftOff(stridePhase);
  double timeUntilFootStrike = stepPatterns_[pIndex].getPhaseLeftUntilStrike(stridePhase);

  //see if we're not in swing mode...
   if (timeUntilFootStrike > timeUntilFootLiftOff)
     return -1;

   double swingPhaseRange = stepPatterns_[pIndex].strikePhase - stepPatterns_[pIndex].liftOffPhase;

   return 1 - timeUntilFootStrike / swingPhaseRange;


  // old working:
  //the absolute phase parameter should be in the range [0-1], so make it so just in case...
//  while (stridePhase < 0) stridePhase += 1; while (stridePhase > 1) stridePhase -= 1;
//  return getRelativePhaseFromAbsolutePhaseInRange(stridePhase, stepPatterns_[pIndex].liftOffPhase, stepPatterns_[pIndex].strikePhase);
}

double loco::GaitPatternFlightPhases::getStancePhaseForLeg(int iLeg, double stridePhase) const {
  int pIndex = getStepPatternIndexForLeg(iLeg);
  if (pIndex == -1) {
    return -1; // changed with new gait pattern from SC
  }

  double timeUntilFootLiftOff = stepPatterns_[pIndex].getPhaseLeftUntilLiftOff(stridePhase);
  double timeUntilFootStrike = stepPatterns_[pIndex].getPhaseLeftUntilStrike(stridePhase);

  if (stepPatterns_[pIndex].strikePhase == stepPatterns_[pIndex].liftOffPhase) {
    return 0.0; // added (Christian)
  }

  //see if we're in swing mode...
  if (timeUntilFootStrike < timeUntilFootLiftOff) {
//    return 0.0>;
    return -1; // changed with new gait pattern from SC
  }


  double swingPhaseRange = stepPatterns_[pIndex].strikePhase - stepPatterns_[pIndex].liftOffPhase;
  double timeSinceFootStrike = 1.0 - timeUntilFootLiftOff - swingPhaseRange;

  return timeSinceFootStrike / (timeSinceFootStrike + timeUntilFootLiftOff);
}

double GaitPatternFlightPhases::getStancePhaseForLeg(int iLeg) {
  return getStancePhaseForLeg(iLeg, cyclePhase_);
}




double GaitPatternFlightPhases::getStanceDuration(int iLeg) {
  return getStanceDuration(iLeg, strideDuration_);
}

double GaitPatternFlightPhases::getSwingDuration(int iLeg, double strideDuration) const {
  return strideDuration - getStanceDuration(iLeg, strideDuration);
}

double GaitPatternFlightPhases::getStanceDuration(int iLeg, double strideDuration) const {
  int pIndex = getStepPatternIndexForLeg(iLeg);
  return (1.0-(stepPatterns_[pIndex].strikePhase - stepPatterns_[pIndex].liftOffPhase)) * strideDuration;
}

unsigned long int loco::GaitPatternFlightPhases::getNGaitCycles() {
  return numGaitCycles_;
}

bool loco::GaitPatternFlightPhases::initialize(double dt) {
  numGaitCycles_ = 0;
  cyclePhase_ = initCyclePhase_;
  isInitialized_ = true;
  return true;
}

bool loco::GaitPatternFlightPhases::isInitialized()
{
  return isInitialized_;
}

bool loco::GaitPatternFlightPhases::advance(double dt) {
  if (strideDuration_ == 0.0) {
    cyclePhase_ = 0.0;
    return true;
  }
  cyclePhase_ += dt/strideDuration_;
  if (cyclePhase_ > 1.0) {
    cyclePhase_ = 0.0;
    numGaitCycles_++;
  }
  return true;

}

bool loco::GaitPatternFlightPhases::shouldBeLegGrounded(int iLeg) {
//  printf("leg %d: stance phase: %f\n",iLeg, getStancePhaseForLeg(iLeg));
//  return (getStancePhaseForLeg(iLeg)!=0.0);
  return (getStancePhaseForLeg(iLeg) >= 0.0); // changed since returns now -1 for swing mode
//  return (getStancePhaseForLeg(iLeg) !=0.0 || footFallPatterns[iLeg].liftOffPhase == footFallPatterns[iLeg].strikePhase); // added (Christian)
}

double GaitPatternFlightPhases::getStridePhase() const {
  return cyclePhase_;
}

void GaitPatternFlightPhases::setStridePhase(double stridePhase) {
  cyclePhase_ =  stridePhase;
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
    if (pElem->QueryDoubleAttribute("cycleDuration", &this->strideDuration_)!=TIXML_SUCCESS) {
      printf("Could not find FlightPhases:cycleDuration\n");
      return false;
    }

    if (pElem->QueryDoubleAttribute("initCyclePhase", &this->initCyclePhase_)!=TIXML_SUCCESS) {
      printf("Could not find FlightPhases:initCyclePhase\n");
      return false;
    }

    /* foot fall pattern */


    stepPatterns_.clear();

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
    stepPatterns_.push_back(FootFallPattern(0, liftOff, touchDown));


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
    stepPatterns_.push_back(FootFallPattern(1, liftOff, touchDown));

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
    stepPatterns_.push_back(FootFallPattern(2, liftOff, touchDown));

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
    stepPatterns_.push_back(FootFallPattern(3, liftOff, touchDown));

    numGaitCycles_ = 0;
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
  int pIndex = getStepPatternIndexForLeg(iLeg);
  assert(pIndex != -1);
  return stepPatterns_[pIndex].liftOffPhase;
}
double GaitPatternFlightPhases::getFootTouchDownPhase(int iLeg) {
  int pIndex = getStepPatternIndexForLeg(iLeg);
  assert(pIndex != -1);
  return stepPatterns_[pIndex].strikePhase;
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
//  printf("Swing phase: %lf\n", swingPhase);
  //the limb is already in swing phase, so the time until the next swing phase starts is one whole stridePhase minus the amount of time it's already spent in swing mode
  if (swingPhase >= 0 && swingPhase <= 1)
    return strideDuration - getTimeSpentInSwing(iLeg, strideDuration, stridePhase);
  //if the limb is in stance phase, then we have as much time until the stance phase ends
  return getTimeLeftInStance(iLeg, strideDuration, stridePhase);
}

int GaitPatternFlightPhases::getNumberOfStanceLegs(double stridePhase) {
  int nStanceLegs = 0;
  for (int i=0; i<stepPatterns_.size();i++) {
    if (getStancePhaseForLeg(i, stridePhase) >= 0.0) {
      nStanceLegs++;
    }
  }
  return nStanceLegs;
}


bool GaitPatternFlightPhases::setToInterpolated(const GaitPatternBase& gaitPattern1, const GaitPatternBase& gaitPattern2, double t) {
  const GaitPatternFlightPhases& gait1 = static_cast<const GaitPatternFlightPhases&>(gaitPattern1);
  const GaitPatternFlightPhases& gait2 = static_cast<const GaitPatternFlightPhases&>(gaitPattern2);

  boundToRange(&t, 0, 1);
  //assume an exact correspondance here (i.e. each foot has the same number of foot fall patterns during a stride).
  //we'll also assume that the order the leg foot pattern is specified is the same...
  strideDuration_ = linearlyInterpolate(gait1.strideDuration_, gait2.strideDuration_, 0, 1, t);

  stepPatterns_.clear();
//  if (gait1.stepPatterns_.size() != gait2.stepPatterns_.size())
//    throw std::runtime_error("Don't know how to interpolated between incompatible foot fall patterns");
//  for (uint i=0;i<gait1.stepPatterns_.size();i++){
//    const FootFallPattern& stepPattern1  = gait1.stepPatterns_[i];
//    int pIndex = getStepPatternIndexForLeg(stepPattern1.legId_);
//    assert(pIndex != 0);
//    const FootFallPattern& stepPattern2 = gait2.stepPatterns_[pIndex];
//    stepPatterns_.push_back(FootFallPattern(stepPattern1.legId_, linearlyInterpolate(stepPattern1.liftOffPhase, stepPattern2.liftOffPhase, 0, 1, t),
//                                               linearlyInterpolate(stepPattern1.strikePhase, stepPattern2.strikePhase, 0, 1, t)));
//  }
//
  if (gait1.stepPatterns_.size() != gait2.stepPatterns_.size())
    throw std::runtime_error("Don't know how to interpolated between incompatible foot fall patterns");
  for (int i=0;i<(int)gait1.stepPatterns_.size();i++){
    stepPatterns_.push_back(FootFallPattern(gait1.stepPatterns_[i].legId_,
        linearlyInterpolate(gait1.stepPatterns_[i].liftOffPhase, gait2.stepPatterns_[i].liftOffPhase, 0, 1, t),
        linearlyInterpolate(gait1.stepPatterns_[i].strikePhase, gait2.stepPatterns_[i].strikePhase, 0, 1, t)));
  }
  return true;
}

void GaitPatternFlightPhases::clear() {
  stepPatterns_.clear();
}

int GaitPatternFlightPhases::getNumberOfLegs() const {
  return (int) stepPatterns_.size();
}

void GaitPatternFlightPhases::addFootFallPattern(int legId, double liftOffPhase, double strikePhase) {
  //ASSUMPTIONS:
  //  - the limb was going to be in swing mode at all times in the interval liftOffPhase and strikePhase
  //  - liftOffPhase happens before strikePhase
  //  - either times can exceed the 0-1 phase range which just means that the foot should already (still) be
  //    in swing mode, but at least one needs to be in the correct range (0-1)

  //see if this limb was added before - if not, add a new one...
  if (liftOffPhase >= strikePhase)
    throw std::runtime_error("addFootFallPattern: liftOffPhase needs to happen before the foot strike.");

  if (strikePhase - liftOffPhase > 1)
    throw std::runtime_error("addFootFallPattern: liftOffPhase and strikePhase should not be offset by more than 1.");

  if ((liftOffPhase < 0 || liftOffPhase > 1) && (strikePhase < 0 || strikePhase > 1))
    throw std::runtime_error("addFootFallPattern: Either liftOffPhase or strikePhase needs to be in the range 0-1.");


  int pIndex = getStepPatternIndexForLeg(legId);
  if (pIndex == -1){
    stepPatterns_.push_back(FootFallPattern(legId, liftOffPhase, strikePhase));
  }else{
    throw std::runtime_error("There is already a footfall pattern for this leg!\n");
  }


}

int GaitPatternFlightPhases::getStepPatternIndexForLeg(int legId) const {
  for (uint i=0;i<stepPatterns_.size();i++)
    if (stepPatterns_[i].legId_ == legId)
      return i;

  return -1;
}

std::ostream& operator << (std::ostream& out, const GaitPatternFlightPhases& gaitPattern) {
  for (uint i=0;i<gaitPattern.stepPatterns_.size();i++) {
    out << "Leg " << gaitPattern.stepPatterns_[i].legId_;
    out << "\tlift-off: " <<  gaitPattern.stepPatterns_[i].liftOffPhase;
    out << "\ttouch-down: " << gaitPattern.stepPatterns_[i].strikePhase;
    out << std::endl;
  }
  return out;
}


} /* namespace loco */
