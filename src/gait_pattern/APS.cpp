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
* @file     APS.cpp
* @author   Christian Gehring
* @date     Jun, 2013
* @version  1.0
* @ingroup
* @brief
*/

#include "loco/gait_pattern/APS.hpp"

#include <cstdio>
#include <cassert>
#include <cmath>
#include <algorithm>

namespace loco {


APS::APS():
	phase_(0.0),
	startTime_(0.0),
	foreCycleDuration_(0.0),
	hindCycleDuration_(0.0),
	foreDutyFactor_(0.0),
	hindDutyFactor_(0.0),
	foreLag_(0.0),
	hindLag_(0.0),
	pairLag_(0.0),
	interpolate_(0.0),
	foreCycleDurationLaw_(APS_VL_NONE),
	hindCycleDurationLaw_(APS_VL_NONE),
	foreDutyFactorLaw_(APS_VL_NONE),
	hindDutyFactorLaw_(APS_VL_NONE),
	foreLagLaw_(APS_VL_NONE),
	hindLagLaw_(APS_VL_NONE),
	pairLagLaw_(APS_VL_NONE)
{

//	foreCycleDurationLaw_ = APS_VL_EXP;
//	foreCycleDurationMinVelocity_ = 0.0;
//	foreCycleDurationMinVelocityValue_ = 0.8;
//	foreCycleDurationMaxVelocity_ = 0.6;
//	foreCycleDurationMaxVelocityValue_ = 0.4;
//	double vel = 0.0;
//	printf("\D = [");
//	for (int i=0; i<10; i++) {
//		setParameters(vel);
//		printf("%lf ", foreCycleDuration_);
//		vel += 0.1;
//	}
//	printf(" ]");
//	printf("\nvel = [");
//	vel = 0.0;
//	for (int i=0; i<10; i++) {
//		printf("%lf ", vel);
//		vel += 0.1;
//	}
//	printf(" ]");
}


APS::APS(double foreCycleDuration, double hindCycleDuration, double foreDutyFactor, double hindDutyFactor, double foreLag, double hindLag, double pairLag) :
phase_(0.0),
startTime_(0.0),
foreCycleDuration_(foreCycleDuration),
hindCycleDuration_(hindCycleDuration),
foreDutyFactor_(foreDutyFactor),
hindDutyFactor_(hindDutyFactor),
foreLag_(foreLag),
hindLag_(hindLag),
pairLag_(pairLag),
interpolate_(0.0),
foreCycleDurationLaw_(APS_VL_NONE),
hindCycleDurationLaw_(APS_VL_NONE),
foreDutyFactorLaw_(APS_VL_NONE),
hindDutyFactorLaw_(APS_VL_NONE),
foreLagLaw_(APS_VL_NONE),
hindLagLaw_(APS_VL_NONE),
pairLagLaw_(APS_VL_NONE)
{

}
APS::~APS(){

}

void APS::getAPSTimesForLeg(int iLeg, double& timeAPSStart, double& timeAPSEnd, double& timeStanceStart, double& timeStanceEnd)
{
	timeAPSStart = getTimeAPSStart();
	timeStanceStart = getTimeStanceStart(iLeg);
	timeStanceEnd = getTimeStanceEnd(iLeg);
	timeAPSEnd = getTimeAPSEnd();

}
double APS::getTimeFootLiftOff(int iLeg) {
	return getTimeStanceEnd(iLeg);
}

double APS::getTimeFootTouchDown(int iLeg) {
	return getTimeStanceStart(iLeg);
}

/*! Stance duration for a steady gait for a given leg
 * @param iLeg	index of the leg  [0,1,2,3]
 * @return	stance duration in seconds
 */
double APS::getStanceDurationForLeg(int iLeg) {
	return getDutyFactorForLeg(iLeg)*getCycleDurationForLeg(iLeg);
}

/*! Swing duration for a steady gait for a given leg
 * @param iLeg	index of the leg  [0,1,2,3]
 * @return	swing duration in seconds
 */
double APS::getSwingDurationForLeg(int iLeg) {
	return (1.0-getDutyFactorForLeg(iLeg))*getCycleDurationForLeg(iLeg);
}

double APS::getDutyFactorForLeg(int iLeg)
{
	double dutyFactor  = 0.0;
	switch (iLeg) {
	case 0:
		dutyFactor = this->foreDutyFactor_;
		break;
	case 1:
		dutyFactor = this->foreDutyFactor_;
		break;
	case 2:
		dutyFactor = this->hindDutyFactor_;
		break;
	case 3:
		dutyFactor = this->hindDutyFactor_;
		break;
	default:
		printf("APS: wrong leg index %d!", iLeg);
	}
	return dutyFactor;

}

double APS::getLagForLeg(int iLeg)
{
	double startLag = 0.0;
	switch (iLeg) {
	case 0:
		startLag = 0.0;
		break;
	case 1:
		startLag = this->foreLag_;
		break;
	case 2:
		startLag = this->pairLag_;
		break;
	case 3:
		startLag = this->pairLag_+this->hindLag_;
		break;
	default:
		printf("APS: wrong leg index %d!", iLeg);
	}
	return startLag;
}

double APS::getLagTimeForLeg(int iLeg)
{
	double lagTime = 0.0;
	switch (iLeg) {
	case 0:
		lagTime = 0.0;
		break;
	case 1:
		lagTime = this->foreLag_*this->foreCycleDuration_;
		break;
	case 2:
		lagTime = this->pairLag_*this->foreCycleDuration_;
		break;
	case 3:
		lagTime = this->pairLag_*this->foreCycleDuration_+this->hindLag_*this->hindCycleDuration_;
		break;
	default:
		printf("APS: wrong leg index %d!", iLeg);
	}
	return lagTime;
}

double APS::getCycleDurationForLeg(int iLeg)
{
	double cycleDuration  = 0.0;
	switch (iLeg) {
	case 0:
		cycleDuration = this->foreCycleDuration_;
		break;
	case 1:
		cycleDuration = this->foreCycleDuration_;
		break;
	case 2:
		cycleDuration = this->hindCycleDuration_;
		break;
	case 3:
		cycleDuration = this->hindCycleDuration_;
		break;
	default:
		printf("APS: wrong leg index %d!", iLeg);
	}
	return cycleDuration;
}

double APS::getTimeAPSStart()
{
	return this->startTime_;
}

double APS::getTimeAPSEnd()
{
	return this->startTime_+this->foreCycleDuration_;
}

double APS::getTimeStanceStart(int iLeg)
{
	return this->startTime_ + getLagTimeForLeg(iLeg);
}

double APS::getTimeStanceEnd(int iLeg)
{
	return this->startTime_  + getLagTimeForLeg(iLeg)+getDutyFactorForLeg(iLeg)*getCycleDurationForLeg(iLeg);
}


void APS::print(){
	printf("startTime: %lf\n", startTime_);
	printf("phase: %lf\n", phase_);
	printf("foreCycleDuration: %lf\n", foreCycleDuration_);
	printf("hindCycleDuration: %lf\n", hindCycleDuration_);
	printf("foreDutyFactor_: %lf\n", foreDutyFactor_);
	printf("hindDutyFactor_: %lf\n",hindDutyFactor_);
	printf("foreLag_: %lf\n", foreLag_);
	printf("hindLag_: %lf\n", hindLag_);
	printf("pairLag_: %lf\n", pairLag_);
	printf("interpolate_: %lf\n", interpolate_);

	printf("timeFootTouchDown: \t");
	for (int i=0;i<4;i++) {
//			printf("%lf ",timeFootTouchDown_[i]);
		printf("%.3lf ",getTimeFootTouchDown(i));
	}
	printf("\n");

	printf("timeFootLiftOff: \t");
	for (int i=0;i<4;i++) {
//			printf("%lf ",timeFootLiftOff_[i]);
		printf("%.3lf ",getTimeFootLiftOff(i));
	}
	printf("\n");
}


void APS::setParameters(double velocity)
{
	foreCycleDuration_ = getValueByLaw(foreCycleDurationLaw_, foreCycleDuration_, velocity, foreCycleDurationMinVelocity_, foreCycleDurationMinVelocityValue_, foreCycleDurationMaxVelocity_, foreCycleDurationMaxVelocityValue_);
	hindCycleDuration_ = getValueByLaw(hindCycleDurationLaw_, hindCycleDuration_, velocity, hindCycleDurationMinVelocity_, hindCycleDurationMinVelocityValue_, hindCycleDurationMaxVelocity_, hindCycleDurationMaxVelocityValue_);
	foreDutyFactor_ = getValueByLaw(foreDutyFactorLaw_, foreDutyFactor_, velocity, foreDutyFactorMinVelocity_, foreDutyFactorMinVelocityValue_, foreDutyFactorMaxVelocity_, foreDutyFactorMaxVelocityValue_);
	hindDutyFactor_ = getValueByLaw(hindDutyFactorLaw_, hindDutyFactor_, velocity, hindDutyFactorMinVelocity_, hindDutyFactorMinVelocityValue_, hindDutyFactorMaxVelocity_, hindDutyFactorMaxVelocityValue_);
	foreLag_ = getValueByLaw(foreLagLaw_, foreLag_, velocity, foreLagMinVelocity_, foreLagMinVelocityValue_, foreLagMaxVelocity_,foreLagMaxVelocityValue_);
	hindLag_ = getValueByLaw(hindLagLaw_, hindLag_, velocity, hindLagMinVelocity_, hindLagMinVelocityValue_, hindLagMaxVelocity_,hindLagMaxVelocityValue_);
	pairLag_ = getValueByLaw(pairLagLaw_, pairLag_, velocity, pairLagMinVelocity_, pairLagMinVelocityValue_, pairLagMaxVelocity_,pairLagMaxVelocityValue_);
}

double APS::getValueByLaw(APS::VelocityLaw law, double currentValue, double velocity, double  minVel, double minVelValue, double maxVel, double maxVelValue) {
	double value = 0.0;
	switch (law) {
		case APS_VL_NONE:
			value = currentValue;
			break;
		case APS_VL_EXP:
			value = getValueByExpLaw(velocity, minVel, minVelValue, maxVel, maxVelValue);
			break;
		case APS_VL_LOG:
			value = getValueByLogLaw(velocity, minVel, minVelValue, maxVel, maxVelValue);
			break;
		case APS_VL_LIN:
			value = getValueByLinLaw(velocity, minVel, minVelValue, maxVel, maxVelValue);
			break;
		default:
			printf("ERROR: getValueByLaw(): unknown law!\n");
			value = 0.0;
			break;
	}
	return value;
}

double APS::getValueByExpLaw(double velocity, double  minVel, double minVelValue, double maxVel, double maxVelValue)
{
	double b =0;
	const double velDiff = exp(-minVel)-exp(-maxVel);
	if (velDiff != 0.0) {
		b = (minVelValue - maxVelValue)/(velDiff);
	}

	const double a = minVelValue-b*exp(-minVel);
	return std::min(std::max(a+b*exp(-std::fabs(velocity)),maxVelValue), minVelValue);
}

double APS::getValueByLinLaw(double velocity, double  minVel, double minVelValue, double maxVel, double maxVelValue)
{
	double b =0;
	const double velDiff = (minVel-maxVel);
	if (velDiff != 0.0) {
		b = (minVelValue - maxVelValue)/(velDiff);
	}
	const double a = minVelValue-b*minVel;
	return std::min(std::max(a+b*std::fabs(velocity),maxVelValue), minVelValue);
}

double APS::getValueByLogLaw(double velocity, double  minVel, double minVelValue, double maxVel, double maxVelValue)
{
	if (minVel==0.0 || maxVel==0.0) {
		printf("Warning: getValueByLogLaw(): log(0)!");
		return getValueByLinLaw(velocity, minVel, minVelValue, maxVel, maxVelValue);
	}
	if (velocity == 0.0) {
		printf("Warning: getValueByLogLaw(): log(0)!");
		return minVelValue;
	}

	double b =0;
	const double velDiff = (log(minVel)-log(maxVel));
	if (velDiff != 0.0) {
		b = (minVelValue - maxVelValue)/(velDiff);
	}
	const double a = minVelValue-b*log(minVel);
	return std::min(std::max(a+b*log(std::fabs(velocity)),maxVelValue), minVelValue);
}


} // namespace loco
