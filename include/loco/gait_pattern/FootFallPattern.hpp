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

	//keep track of the relative phase when the foot lifts off the ground...
	double liftOffPhase;
	//and the relative phase when the foot strikes the ground...
	double strikePhase;

	FootFallPattern(double fLiftOff, double fStrike){
		liftOffPhase = fLiftOff;
		strikePhase = fStrike;
	}

	/*!
	 * @param absolutePhase 	current state of the stride phase in [0,1]
	 * @return phase left until foot lift off (which can happen in next stride)
	 */
	double getPhaseLeftUntilFootLiftOff(double absolutePhase) const {
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

	double getPhaseLeftUntilFootStrike(double absolutePhase) const {
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
