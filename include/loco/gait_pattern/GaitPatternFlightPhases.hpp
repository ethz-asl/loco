/*****************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Christian Gehring, Péter Fankhauser, C. Dario Bellicoso, Stelian Coros
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*
 * GaitPatternFlightPhases.hpp
 *
 *  Created on: Mar 14, 2014
 *      Author: gech
 */

#ifndef LOCO_GAITPATTERNFLIGHTPHASES_HPP_
#define LOCO_GAITPATTERNFLIGHTPHASES_HPP_

#include "loco/gait_pattern/GaitPatternBase.hpp"
#include "loco/gait_pattern/FootFallPattern.hpp"

#include "loco/common/TorsoBase.hpp"
#include "loco/common/LegGroup.hpp"

#include <vector>

namespace loco {

class GaitPatternFlightPhases: public GaitPatternBase {
 public:
  GaitPatternFlightPhases(LegGroup* legs, TorsoBase* torso);
  virtual ~GaitPatternFlightPhases();

  /*! @returns the stride duration in seconds
     */
    virtual double getStrideDuration();

    /*! Sets the stride duration
     * @param strideDuration  stride duration in seconds
     */
    virtual void setStrideDuration(double strideDuration);

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

    virtual double getTimeLeftInStance(int iLeg, double strideDuration, double stridePhase) const;
    virtual double getTimeLeftInSwing(int iLeg, double strideDuration, double stridePhase) const;

    virtual double getTimeSpentInStance(int iLeg, double strideDuration, double stridePhase) const;
    virtual double getTimeSpentInSwing(int iLeg, double strideDuration, double stridePhase) const;

    virtual double getTimeUntilNextStancePhase(int iLeg, double strideDuration, double stridePhase) const;
    virtual double getTimeUntilNextSwingPhase(int iLeg, double strideDuration, double stridePhase) const;

    /*! @returns number of legs that are in stance mode
     * @param stridePhase   stride phase
     */
    virtual int getNumberOfStanceLegs(double stridePhase);

    /*!  @returns number of gait cycles
     */
    virtual unsigned long int getNGaitCycles();

    virtual bool initialize(double dt);

    bool isInitialized();

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

    virtual bool loadParameters(const TiXmlHandle& handle);


    double getFootLiftOffPhase(int iLeg);
    double getFootTouchDownPhase(int iLeg);

    /*!
      computed an interpolated version of the two gaits passed in as parameters.
      if t is 0, the current gait is set to gait1, 1 -> gait 2, and values in between
      correspond to interpolated gaits.
    */
    virtual bool setToInterpolated(const GaitPatternBase& gaitPattern1, const GaitPatternBase& gaitPattern2, double t);

    void clear();

    int getNumberOfLegs() const;

    void addFootFallPattern(int legId, double liftOffPhase, double strikePhase);

    friend std::ostream& operator << (std::ostream& out, const GaitPatternFlightPhases& gaitPattern);
 private:
    bool isInitialized_;

    double cyclePhase_;

    //! this is how long it should take from the time one particular foot leaves the ground, until it leaves the ground again next in seconds.
    double strideDuration_;

    //! the gait should start at this stride phase in [0,1]
    double initCyclePhase_;

    //! number of gait cycles since start
    unsigned long int numGaitCycles_;

    std::vector<FootFallPattern> stepPatterns_;

 protected:
    /*!
      Given a "circular" domain:

      (and 0.95 is in the range [-0.1, 0.1], and 0.05 is in the range [0.9, 1.1], for instance).

      start and end must satisfy the condition:
        start >= 0 || end <= 1
      and phase must satisfy:
        phase >= 0 && phase <= 1

      This method returns a number between 0 and 1 if phase is in the range start-end
      It returns 0 if phase is just at the start of the interval, 1 if it's at the end (interpolated
      values in between) and -1 if the phase is not in the range.
    */
    double getRelativePhaseFromAbsolutePhaseInRange(double phase, double start, double end) const;
    virtual void updateGaitPattern(double dt);

    LegGroup* legs_;
    TorsoBase* torso_;


    int getStepPatternIndexForLeg(int legId) const;
};

} /* namespace loco */

#endif /* LOCO_GAITPATTERNFLIGHTPHASES_HPP_ */
