/*
 * GaitTransition.cpp
 *
 *  Created on: Nov 29, 2012
 *      Author: starleth
 */

#include "loco/gait_switcher/GaitTransition.hpp"

namespace loco {

GaitTransition::GaitTransition():
startLocomotionSettings(0),
endLocomotionSettings(0),
timeInterval(-1),
//stridePhaseTrigger(-1),
smallerSpeedTrigger(-1),
largerSpeedTrigger(-1)
{


}

GaitTransition::~GaitTransition() {

}

} // end namespace loco
