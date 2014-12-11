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
* @file 	FootFallPattern.h
* @author 	Christian Gehring, Stelian Coros
* @date		Jun 14, 2012
* @version 	1.0
* @ingroup 	robotController
* @brief
*/
#ifndef LOCO_FOOTFALLPATTERN_HPP_
#define LOCO_FOOTFALLPATTERN_HPP_

namespace loco {

class FootFallPattern {
public:
  //! Identifier of the leg this pattern belongs to
  int legId_;
	//keep track of the relative phase when the foot lifts off the ground...
	double liftOffPhase;
	//and the relative phase when the foot strikes the ground...
	double strikePhase;

	FootFallPattern(int legId, double fLiftOff, double fStrike) :
	  legId_(legId),
	  liftOffPhase(fLiftOff),
	  strikePhase(fStrike)
	{
	}

	/*!
	 * @param absolutePhase 	current state of the stride phase in [0,1]
	 * @return phase left until foot lift off (which can happen in next stride)
	 */
	double getPhaseLeftUntilLiftOff(double absolutePhase) const {
		assert(!(absolutePhase < 0 || absolutePhase > 1)); //"no reason why absolute phase should be outside the range [0, 1]");
		if (liftOffPhase == strikePhase) { // added (Christian)
			return 1;
		}

		double start = liftOffPhase;
		//we need the line below twice: if footLiftOff is -0.1, and phase = 0.95 for instance...
		if (start < absolutePhase) start += 1;
		if (start < absolutePhase) start += 1;

		double result = start - absolutePhase;
		assert(!(result < 0 || result > 1)); //"Bug in getPhaseLeftUntilFootLiftOff!!!";
		return result;
	}

	double getPhaseLeftUntilStrike(double absolutePhase) const {
		assert(!(absolutePhase < 0 || absolutePhase > 1)); //"no reason why absolute phase should be outside the range [0, 1]");
		if (liftOffPhase == strikePhase) { // added (Christian)
			return 0;
		}

		double end = strikePhase;
		if (end < absolutePhase) end += 1;
		if (end < absolutePhase) end += 1;
		double result = end - absolutePhase;
		if (result > 1) result -= 1;
		assert(!(result < 0 || result > 1)); //"Bug in getPhaseLeftUntilFootStrike!!!"
		return result;
	}

};

} // namespace loco

#endif /* LOCO_FOOTFALLPATTERN_HPP_ */
