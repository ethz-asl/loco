/*
 * GaitTransition.hpp
 *
 *  Created on: Nov 29, 2012
 *      Author: starleth
 */

#ifndef LOCO_GAITTRANSITION_HPP_
#define LOCO_GAITTRANSITION_HPP_

#include "loco/locomotion_controller/LocomotionControllerDynamicGaitDefault.hpp"
#include <string>
#include "loco/gait_pattern/APS.hpp"
#include <list>

namespace loco {

class GaitTransition {
public:
	GaitTransition();
	virtual ~GaitTransition();
	LocomotionControllerDynamicGaitDefault* startLocomotionSettings;
	LocomotionControllerDynamicGaitDefault* endLocomotionSettings;

	double timeInterval;
//	double stridePhaseTrigger;
	double smallerSpeedTrigger;
	double largerSpeedTrigger;
	double stridePhaseTrigger;
	std::string startName;
	std::string endName;

	/* APS */
//	int nCycles_;

	std::list<APS> listAPS_;


};

} // end namespace loco

#endif /* GAITTRANSITION_HPP_ */
