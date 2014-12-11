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
/*!
* @file     APS.hpp
* @author   Christian Gehring
* @date     Jun, 2013
* @version  1.0
* @ingroup
* @brief
*/

#ifndef LOCO_APS_HPP_
#define LOCO_APS_HPP_

namespace loco {


class APS {
public:
	APS();
	APS(double foreCycleDuration, double hindCycleDuration, double foreDutyFactor, double hindDutyFactor, double foreLag, double hindLag, double pairLag);
	virtual ~APS();

public:
	enum VelocityLaw {
		APS_VL_NONE=0,
		APS_VL_EXP,
		APS_VL_LOG,
		APS_VL_LIN
	};
public:
	double phase_;
	double startTime_;
	double foreCycleDuration_;
	double hindCycleDuration_;
	double foreDutyFactor_;
	double hindDutyFactor_;
	double foreLag_;
	double hindLag_;
	double pairLag_;
	double interpolate_;

	APS::VelocityLaw foreCycleDurationLaw_;
	APS::VelocityLaw hindCycleDurationLaw_;
	APS::VelocityLaw foreDutyFactorLaw_;
	APS::VelocityLaw hindDutyFactorLaw_;
	APS::VelocityLaw foreLagLaw_;
	APS::VelocityLaw hindLagLaw_;
	APS::VelocityLaw pairLagLaw_;

	double foreCycleDurationMinVelocity_;
	double foreCycleDurationMinVelocityValue_;
	double foreCycleDurationMaxVelocity_;
	double foreCycleDurationMaxVelocityValue_;

	double hindCycleDurationMinVelocity_;
	double hindCycleDurationMinVelocityValue_;
	double hindCycleDurationMaxVelocity_;
	double hindCycleDurationMaxVelocityValue_;

	double foreDutyFactorMinVelocity_;
	double foreDutyFactorMinVelocityValue_;
	double foreDutyFactorMaxVelocity_;
	double foreDutyFactorMaxVelocityValue_;

	double hindDutyFactorMinVelocity_;
	double hindDutyFactorMinVelocityValue_;
	double hindDutyFactorMaxVelocity_;
	double hindDutyFactorMaxVelocityValue_;

	double foreLagMinVelocity_;
	double foreLagMinVelocityValue_;
	double foreLagMaxVelocity_;
	double foreLagMaxVelocityValue_;

	double hindLagMinVelocity_;
	double hindLagMinVelocityValue_;
	double hindLagMaxVelocity_;
	double hindLagMaxVelocityValue_;

	double pairLagMinVelocity_;
	double pairLagMinVelocityValue_;
	double pairLagMaxVelocity_;
	double pairLagMaxVelocityValue_;
public:
	void getAPSTimesForLeg(int iLeg, double& timeAPSStart, double& timeAPSEnd, double& timeStanceStart, double& timeStanceEnd);
	double getTimeFootLiftOff(int iLeg);

	double getTimeFootTouchDown(int iLeg);

	/*! Stance duration for a steady gait for a given leg
	 * @param iLeg	index of the leg  [0,1,2,3]
	 * @return	stance duration in seconds
	 */
	double getStanceDurationForLeg(int iLeg);

	/*! Swing duration for a steady gait for a given leg
	 * @param iLeg	index of the leg  [0,1,2,3]
	 * @return	swing duration in seconds
	 */
	double getSwingDurationForLeg(int iLeg);
	double getDutyFactorForLeg(int iLeg);
	double getLagForLeg(int iLeg);
	double getLagTimeForLeg(int iLeg);
	double getCycleDurationForLeg(int iLeg);

	double getTimeAPSStart();

	double getTimeAPSEnd();
	double getTimeStanceStart(int iLeg);

	double getTimeStanceEnd(int iLeg);

	void setParameters(double velocity);
	double getValueByLaw(APS::VelocityLaw law, double currentValue, double velocity, double  minVel, double minVelValue, double maxVel, double maxVelValue);
	double getValueByExpLaw(double velocity, double  minVel, double minVelValue, double maxVel, double maxVelValue);
	double getValueByLinLaw(double velocity, double  minVel, double minVelValue, double maxVel, double maxVelValue);
	double getValueByLogLaw(double velocity, double  minVel, double minVelValue, double maxVel, double maxVelValue);

	void print();

};

}


#endif /* LOCO_APS_HPP_ */
