/*!
* @file     GaitPatternAPS.hpp
* @author   Christian Gehring
* @date     Jun, 2013
* @version  1.0
* @ingroup
* @brief
*/

#include "loco/gait_pattern/GaitPatternAPS.hpp"


#include "tinyxml.h"

#include <cmath>
#include <vector>

namespace loco {

GaitPatternAPS::GaitPatternAPS():velocity_(0.0) {

	numGaitCycles = 0;

	for (int iLeg=0; iLeg<4; iLeg++)  {
		stancePhases_[iLeg] = 0.0;
		swingPhases_[iLeg] = -1.0;
	}

}

GaitPatternAPS::~GaitPatternAPS() {


}



double GaitPatternAPS::getSwingPhaseForLeg(int iLeg) {
	return GaitAPS::getSwingPhase(iLeg);

}


double GaitPatternAPS::getStanceDuration(int iLeg) {
	return GaitAPS::getStanceDuration(iLeg);
}

double GaitPatternAPS::getStrideDuration()
{
	return GaitAPS::getStrideDuration();
}

void GaitPatternAPS::setStrideDuration(double strideDuration)
{
	GaitAPS::setStrideDuration(strideDuration);
}


double GaitPatternAPS::getStancePhaseForLeg(int iLeg){
	return getStancePhase(iLeg);

}



bool GaitPatternAPS::loadParametersFromXML(TiXmlHandle &hParameterSet, double dt)
{
	double value, liftOff, touchDown;
	int iLeg;
	TiXmlElement* pElem;

	/* gait pattern */
	pElem = hParameterSet.FirstChild("GaitPattern").Element();
	if (!pElem) {
		printf("Could not find GaitPattern\n");
		return false;
	}



	/* APS */
	APS newAPS;

	pElem = hParameterSet.FirstChild("GaitPattern").FirstChild("APS").Element();
	if(!pElem) {
		printf("Could not find GaitPattern:APS\n");
		return false;
	}

	pElem = hParameterSet.FirstChild("GaitPattern").FirstChild("APS").FirstChild("GaitDiagram").Element();
	if(!pElem) {
		printf("Could not find GaitPattern:APS:GaitDiagram\n");
		return false;
	}
	if (pElem->QueryDoubleAttribute("cycleDuration", &newAPS.foreCycleDuration_)!=TIXML_SUCCESS) {
		printf("Could not find GaitPattern:APS:GaitDiagram:cycleDuration\n");
		return false;
	}
	newAPS.hindCycleDuration_ = newAPS.foreCycleDuration_;


	if (pElem->QueryDoubleAttribute("foreLag", &newAPS.foreLag_)!=TIXML_SUCCESS) {
		printf("Could not find GaitPattern:APS:GaitDiagram:foreLag\n");
		return false;
	}
	if (pElem->QueryDoubleAttribute("hindLag", &newAPS.hindLag_)!=TIXML_SUCCESS) {
		printf("Could not find GaitPattern:APS:GaitDiagram:hindLag\n");
		return false;
	}
	if (pElem->QueryDoubleAttribute("pairLag", &newAPS.pairLag_)!=TIXML_SUCCESS) {
		printf("Could not find GaitPattern:APS:GaitDiagram:pairLag\n");
		return false;
	}
	if (pElem->QueryDoubleAttribute("foreDutyFactor", &newAPS.foreDutyFactor_)!=TIXML_SUCCESS) {
		printf("Could not find GaitPattern:APS:GaitDiagram:foreDutyFactor\n");
		return false;
	}
	if (pElem->QueryDoubleAttribute("hindDutyFactor", &newAPS.hindDutyFactor_)!=TIXML_SUCCESS) {
		printf("Could not find GaitPattern:APS:GaitDiagram:hindDutyFactor\n");
		return false;
	}


	pElem = hParameterSet.FirstChild("GaitPattern").FirstChild("APS").FirstChild("CycleDuration").Element();
	if(pElem) {
		if (pElem->QueryDoubleAttribute("minVel", &newAPS.foreCycleDurationMinVelocity_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:CycleDuration:minVel\n");
			return false;
		}
		newAPS.hindCycleDurationMinVelocity_ = newAPS.foreCycleDurationMinVelocity_;

		if (pElem->QueryDoubleAttribute("minVelValue", &newAPS.foreCycleDurationMinVelocityValue_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:CycleDuration:minVelValue\n");
			return false;
		}
		newAPS.hindCycleDurationMinVelocityValue_ = newAPS.foreCycleDurationMinVelocityValue_;

		if (pElem->QueryDoubleAttribute("maxVel", &newAPS.foreCycleDurationMaxVelocity_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:CycleDuration:maxVel\n");
			return false;
		}
		newAPS.hindCycleDurationMaxVelocity_ = newAPS.foreCycleDurationMaxVelocity_;

		if (pElem->QueryDoubleAttribute("maxVelValue", &newAPS.foreCycleDurationMaxVelocityValue_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:CycleDuration:maxVelValue\n");
			return false;
		}
		newAPS.hindCycleDurationMaxVelocityValue_ = newAPS.foreCycleDurationMaxVelocityValue_;

		std::string law;
		if (pElem->QueryStringAttribute("law", &law)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:CycleDuration:law\n");
			return false;
		}
		if (law == "none") {
			newAPS.foreCycleDurationLaw_ = APS::APS_VL_NONE;
			newAPS.hindCycleDurationLaw_ = APS::APS_VL_NONE;
		} else  if(law == "lin") {
			newAPS.foreCycleDurationLaw_ = APS::APS_VL_LIN;
			newAPS.hindCycleDurationLaw_ = APS::APS_VL_LIN;
		} else  if(law == "log") {
			newAPS.foreCycleDurationLaw_ = APS::APS_VL_LOG;
			newAPS.hindCycleDurationLaw_ = APS::APS_VL_LOG;
		} else  if(law == "exp") {
			newAPS.foreCycleDurationLaw_ = APS::APS_VL_EXP;
			newAPS.hindCycleDurationLaw_ = APS::APS_VL_EXP;
		}
	}


	pElem = hParameterSet.FirstChild("GaitPattern").FirstChild("APS").FirstChild("ForeDutyFactor").Element();
	if(pElem) {
		if (pElem->QueryDoubleAttribute("minVel", &newAPS.foreDutyFactorMinVelocity_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:ForeDutyFactor:minVel\n");
			return false;
		}

		if (pElem->QueryDoubleAttribute("minVelValue", &newAPS.foreDutyFactorMinVelocityValue_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:ForeDutyFactor:minVelValue\n");
			return false;
		}

		if (pElem->QueryDoubleAttribute("maxVel", &newAPS.foreDutyFactorMaxVelocity_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:ForeDutyFactor:maxVel\n");
			return false;
		}

		if (pElem->QueryDoubleAttribute("maxVelValue", &newAPS.foreDutyFactorMaxVelocityValue_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:ForeDutyFactor:maxVelValue\n");
			return false;
		}

		std::string law;
		if (pElem->QueryStringAttribute("law", &law)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:ForeDutyFactor:law\n");
			return false;
		}
		if (law == "none") {
			newAPS.foreDutyFactorLaw_ = APS::APS_VL_NONE;
		} else  if(law == "lin") {
			newAPS.foreDutyFactorLaw_ = APS::APS_VL_LIN;
		} else  if(law == "log") {
			newAPS.foreDutyFactorLaw_ = APS::APS_VL_LOG;
		} else  if(law == "exp") {
			newAPS.foreDutyFactorLaw_ = APS::APS_VL_EXP;
		}
	}


	pElem = hParameterSet.FirstChild("GaitPattern").FirstChild("APS").FirstChild("HindDutyFactor").Element();
	if(pElem) {
		if (pElem->QueryDoubleAttribute("minVel", &newAPS.hindDutyFactorMinVelocity_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:HindDutyFactor:minVel\n");
			return false;
		}

		if (pElem->QueryDoubleAttribute("minVelValue", &newAPS.hindDutyFactorMinVelocityValue_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:HindDutyFactor:minVelValue\n");
			return false;
		}

		if (pElem->QueryDoubleAttribute("maxVel", &newAPS.hindDutyFactorMaxVelocity_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:HindDutyFactor:maxVel\n");
			return false;
		}

		if (pElem->QueryDoubleAttribute("maxVelValue", &newAPS.hindDutyFactorMaxVelocityValue_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:HindDutyFactor:maxVelValue\n");
			return false;
		}

		std::string law;
		if (pElem->QueryStringAttribute("law", &law)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:HindDutyFactor:law\n");
			return false;
		}
		if (law == "none") {
			newAPS.hindDutyFactorLaw_ = APS::APS_VL_NONE;
		} else  if(law == "lin") {
			newAPS.hindDutyFactorLaw_ = APS::APS_VL_LIN;
		} else  if(law == "log") {
			newAPS.hindDutyFactorLaw_ = APS::APS_VL_LOG;
		} else  if(law == "exp") {
			newAPS.hindDutyFactorLaw_ = APS::APS_VL_EXP;
		}
	}

	pElem = hParameterSet.FirstChild("GaitPattern").FirstChild("APS").FirstChild("PairLag").Element();
	if(pElem) {
		if (pElem->QueryDoubleAttribute("minVel", &newAPS.pairLagMinVelocity_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:PairLag:minVel\n");
			return false;
		}

		if (pElem->QueryDoubleAttribute("minVelValue", &newAPS.pairLagMinVelocityValue_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:PairLag:minVelValue\n");
			return false;
		}

		if (pElem->QueryDoubleAttribute("maxVel", &newAPS.pairLagMaxVelocity_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:PairLag:maxVel\n");
			return false;
		}

		if (pElem->QueryDoubleAttribute("maxVelValue", &newAPS.pairLagMaxVelocityValue_)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:PairLag:maxVelValue\n");
			return false;
		}

		std::string law;
		if (pElem->QueryStringAttribute("law", &law)!=TIXML_SUCCESS) {
			printf("Could not find GaitPattern:APS:GaitDiagram:PairLag:law\n");
			return false;
		}
		if (law == "none") {
			newAPS.pairLagLaw_ = APS::APS_VL_NONE;
		} else  if(law == "lin") {
			newAPS.pairLagLaw_ = APS::APS_VL_LIN;
		} else  if(law == "log") {
			newAPS.pairLagLaw_ = APS::APS_VL_LOG;
		} else  if(law == "exp") {
			newAPS.pairLagLaw_ = APS::APS_VL_EXP;
		}
	}


	GaitAPS::initAPS(newAPS, dt);



	return true;
}

bool GaitPatternAPS::initialize(const APS& aps, double dt) {
  GaitAPS::initAPS(aps, dt);

  return true;
}

bool GaitPatternAPS::saveParametersInXML(TiXmlHandle &hParameterSet)
{

	TiXmlElement* pElem;

	/* gait pattern */
	pElem = hParameterSet.FirstChild("GaitPattern").Element();
	if (!pElem) {
		printf("Could not find GaitPattern\n");
		return false;
	}



	return true;
}


unsigned long int GaitPatternAPS::getNGaitCycles()
{
	return GaitAPS::getNumGaitCycles();
}




void GaitPatternAPS::setVelocity(double value)
{
	velocity_ = value;
	for (int iLeg=0; iLeg<4; iLeg++) {
//		currentAPS[iLeg]->setParameters(value);
		nextAPS[iLeg]->setParameters(value);
		nextNextAPS[iLeg]->setParameters(value);
	}
}

double GaitPatternAPS::getVelocity()
{
	return velocity_;
}

void GaitPatternAPS::advance(double dt) {
  GaitAPS::advance(dt);
}

bool GaitPatternAPS::shouldBeLegGrounded(int iLeg) {
  GaitAPS::shouldBeGrounded(iLeg);
}

} // namespace loco
