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
* @file     GaitAPS.hpp
* @author   Christian Gehring
* @date     Jun, 2013
* @version  1.0
* @ingroup
* @brief
*/

#ifndef LOCO_GAITAPS_HPP_
#define LOCO_GAITAPS_HPP_

#include "loco/gait_pattern/APS.hpp"
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
	void initAPS(const APS& aps, double dt);

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
	virtual bool advance(double dt);

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
	const APS& getCurrentAPS() const;
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

	double getStridePhase() const;

	void setStridePhase(double stridePhase);

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
