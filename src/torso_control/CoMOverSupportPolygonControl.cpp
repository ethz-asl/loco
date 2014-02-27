/*!
 * @file 	    CoMOverSupportPolygonControl.cpp
 * @author 	  Christian Gehring, Stelian Coros
 * @date		  Jul 17, 2012
 * @version 	1.0
 * @ingroup
 * @brief
 */

#include "loco/torso_control/CoMOverSupportPolygonControl.hpp"
#include <stdio.h>
#include <Eigen/Dense>
#include "loco/temp_helpers/math.hpp"

using namespace std;

namespace loco {

CoMOverSupportPolygonControl::CoMOverSupportPolygonControl(LegGroup* legs) :
legs_(legs)
{

	// some example values
	spMinLegWeight = 0.3200;
	spStPShiftAwayFromLegStart = 0.7;
	spSwPShiftTowardsLegStart = 0.7;
	sagittalOffset = 0;
	coronalOffset = 0;
}

CoMOverSupportPolygonControl::~CoMOverSupportPolygonControl() {

}

Eigen::Vector3d CoMOverSupportPolygonControl::getPositionErrorVectorInWorldFrame() {
  const int nLegs = legs_->size();
  double legWeights[nLegs];
  for (int i=0;i<nLegs;i++) {
    legWeights[i] = 1.0;
  }

  double sumWeights = 0;
  int nStanceLegs = 0;
  int iLeg=0;
  for ( auto leg : *legs_) {
    if (leg->isInStanceMode()) {
      double t = 1 - mapTo01Range(leg->getStancePhase(),spStPShiftAwayFromLegStart, 1.0);
      t = linearlyInterpolate(spMinLegWeight, 1, 0, 1, t);
      legWeights[iLeg] = t;
      nStanceLegs++;
    } else {
      double t = mapTo01Range(leg->getSwingPhase(), spSwPShiftTowardsLegStart, 1.0);
      t = linearlyInterpolate(spMinLegWeight, 1, 0, 1, t);
      legWeights[iLeg] = t;
    }
    sumWeights += legWeights[iLeg];
    iLeg++;
  }


  Eigen::Vector3d defaultTarget = Eigen::Vector3d::Zero();
  Eigen::Vector3d comTarget  = Eigen::Vector3d::Zero();

  iLeg=0;
  for (auto leg : *legs_) {
    //	            tprintf("leg %d(%s): stanceMode: %lf, swingMode: %lf. Weight:%lf\n", i, leg->getName(), leg->getStancePhase(),leg->getSwingPhase(), legWeights[i]);
    const Eigen::Vector3d footPositionCSw = leg->getFootPositionInWorldFrame();

    if (sumWeights != 0) {
      comTarget += footPositionCSw*legWeights[iLeg]/sumWeights;
    }
    defaultTarget += footPositionCSw*0.25;
    iLeg++;
  }


  return defaultTarget + comTarget+Eigen::Vector3d(sagittalOffset, coronalOffset, 0.0);

}

bool CoMOverSupportPolygonControl::loadParametersFromXML(TiXmlHandle &hParameterSet) {

	TiXmlElement* pElem;

	pElem = hParameterSet.FirstChild("SupportPolygon").Element();
	if (!pElem) {
		printf("Could not find SupportPolygon\n");
		return false;
	}

	if (pElem->QueryDoubleAttribute("minLegWeight", &this->spMinLegWeight)
			!= TIXML_SUCCESS) {
		printf("Could not find SupportPolygon:minLegWeight!\n");
		return false;
	}

	if (pElem->QueryDoubleAttribute("stPShiftAwayFromLegStart",
			&this->spStPShiftAwayFromLegStart) != TIXML_SUCCESS) {
		printf("Could not find SupportPolygon:stPShiftAwayFromLegStart!\n");
		return false;
	}

	if (pElem->QueryDoubleAttribute("swPShiftTowardsLegStart",
			&this->spSwPShiftTowardsLegStart) != TIXML_SUCCESS) {
		printf("Could not find SupportPolygon:swPShiftTowardsLegStart!\n");
		return false;
	}
	pElem = hParameterSet.FirstChild("SupportPolygon").FirstChild("Offset").Element();
	if (pElem) {
		if (pElem->QueryDoubleAttribute("sagittalOffset",
				&this->sagittalOffset) != TIXML_SUCCESS) {
			sagittalOffset = 0.0;
		}

		if (pElem->QueryDoubleAttribute("coronalOffset",
				&this->coronalOffset) != TIXML_SUCCESS) {
			coronalOffset = 0.0;
		}

	}

	return true;

}

bool CoMOverSupportPolygonControl::saveParametersInXML(TiXmlHandle &hParameterSet) {

	double value;
	TiXmlElement* pElem;

	pElem = hParameterSet.FirstChild("SupportPolygon").Element();
	if (!pElem) {
		printf("Could not find SupportPolygon\n");
		return false;
	}
	pElem->SetDoubleAttribute("minLegWeight", this->spMinLegWeight);
	pElem->SetDoubleAttribute("stPShiftAwayFromLegStart",
			this->spStPShiftAwayFromLegStart);
	pElem->SetDoubleAttribute("swPShiftTowardsLegStart",
			this->spSwPShiftTowardsLegStart);

	return true;

}

bool CoMOverSupportPolygonControl::setToInterpolated(const CoMOverSupportPolygonControl& supportPolygon1,
		const CoMOverSupportPolygonControl& supportPolygon2, double t) {
	this->spMinLegWeight = linearlyInterpolate(supportPolygon1.spMinLegWeight,
			supportPolygon2.spMinLegWeight, 0, 1, t);
	this->spStPShiftAwayFromLegStart = linearlyInterpolate(
			supportPolygon1.spStPShiftAwayFromLegStart,
			supportPolygon2.spStPShiftAwayFromLegStart, 0, 1, t);
	this->spSwPShiftTowardsLegStart = linearlyInterpolate(
			supportPolygon1.spSwPShiftTowardsLegStart,
			supportPolygon2.spSwPShiftTowardsLegStart, 0, 1, t);

	this->sagittalOffset = linearlyInterpolate(
			supportPolygon1.sagittalOffset,
			supportPolygon2.sagittalOffset, 0, 1, t);

	this->coronalOffset = linearlyInterpolate(
			supportPolygon1.coronalOffset,
			supportPolygon2.coronalOffset, 0, 1, t);

	return true;
}





} // namespace loco
