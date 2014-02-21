/*
 * GaitAPS.hpp
 *
 *  Created on: Aug 22, 2013
 *      Author: gech
 */
#ifndef LOCO_GAITAPS_HPP_
#define LOCO_GAITAPS_HPP_

#include "APS.hpp"
#include <list>
#include <cassert>

namespace loco {

class GaitAPS {
public:
	typedef std::list<APS>::iterator APSIterator;
public:
	GaitAPS();
	virtual ~GaitAPS();

	/*!
	 *
	 * @param aps	initial APS
	 * @param dt		time step in seconds
	 */
	void initAPS(APS &aps, double dt);

	/*! Gets the swing phase for a given leg
	 * The swing phase is in [0,1] if the leg is in swing mode and -1 if it is in stance mode
	 * @param iLeg	index of the leg [0, 1, 2, 3]
	 * @return phase
	 */
	double getSwingPhase(int iLeg);

	/*! Gets the stance phase for a given leg
	 * The stance phase is in [0,1], whereas 0 means it is in swing mode
	 * @param iLeg	index of the leg [0, 1, 2, 3]
	 * @return
	 */
	double getStancePhase(int iLeg);

	/*! Gets the stance duration for a given leg
	 * The stance duration is defined as the duty-factor times the cycle duration of the current APS of this leg.
	 * @param iLeg	index of the leg [0, 1, 2, 3]
	 * @return	duration in seconds
	 */
	double getStanceDuration(int iLeg);

	/*! Gets the swing duration for a given leg
	 * The swing duration is defined as the interval between the lift-off of the current APS of this leg and the touch-down of the next APS of this leg.
	 * @param iLeg		index of the leg [0, 1, 2 , 3]
	 * @return duration in seconds
	 */
	double getSwingDuration(int iLeg);

	/*! Get number of cycles since start
	 * @return
	 */
	virtual unsigned long int getNumGaitCycles();

	/*! Gets the stride duration of the current APS
	 * 	The stride duration is defined as the cycle duration of the first leg of the current APS
	 * @return	duration in seconds
	 */
	virtual double getStrideDuration();

	/*! Sets the stride duration
	 * WARNING: not tested
	 * @param strideDuration
	 */
	virtual void setStrideDuration(double strideDuration);

	/*! Advance APS in time
	 *
	 * @param dt	time step [s]
	 */
	virtual void advance(double dt);

	/*! Gets the time in seconds
	 * @return
	 */
	virtual double getTime();


	virtual APS* getLastAPS();
	virtual APS* getFirstAPS();
	void addAPS(APS aps);
	void popLastAPS();
	void print();
	void printTDandLO();

	APS* getCurrentAPS();
	APS* getNextAPS();
	APS* getCurrentAPS(int iLeg);
	APS* getNextAPS(int iLeg);
	APS* getNextNextAPS(int iLeg);
	APS* getPreviousAPS(int iLeg);
	APS* getPreviousPreviousAPS(int iLeg);

	APSIterator getIteratorBegin();
	APSIterator getIteratorEnd();

	void resetInterpolation();
	/*! Gets number of APS in list
	 *
	 * @return
	 */
	unsigned int getAPSSize();

	/*! Returns true if the leg should be grounded according to the APS
	 * @param iLeg	index of the leg [0,1,2,3]
	 * @return true if leg should be grounded
	 */
	bool shouldBeGrounded(int iLeg);
protected:
	//! number of gait cycles since start
	unsigned long int numGaitCycles;

	//! simulated time
	double time_;

protected:
	//! list of APS for each cycle
	std::list<APS> listAPS_;

	//! current APS of each leg  [LF RF LH RH]
	APSIterator currentAPS[4];
	//! next APS of each leg  [LF RF LH RH]
	APSIterator nextAPS[4];
	//! second next APS of each leg  [LF RF LH RH]
	APSIterator nextNextAPS[4];

	//! previous APS of each leg  [LF RF LH RH]
	APSIterator prevAPS[4];
public:
	//! stance phase in [0, 1] for each leg [LF RF LH RH]
	double stancePhases_[4];

	//! swing phase in {-1, [0, 1]} for each leg [LF RF LH RH]
	double swingPhases_[4];


};

} // namespace loco

#endif /* LOCO_GAITAPS_HPP_ */
