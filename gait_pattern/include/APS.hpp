/*
 * APS.hpp
 *
 *  Created on: Aug 13, 2013
 *      Author: gech
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
