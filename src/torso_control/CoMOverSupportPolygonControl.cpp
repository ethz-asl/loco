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

	// trot
	minSwingLegWeight_ = 0.15;
	startShiftAwayFromLegAtStancePhase_ = 0.7;
	startShiftTowardsLegAtSwingPhase_ = 0.7;
	headingOffset_ = 0;
	lateralOffset_ = 0;
}

CoMOverSupportPolygonControl::~CoMOverSupportPolygonControl() {

}


void CoMOverSupportPolygonControl::advance(double dt) {
  const int nLegs = legs_->size();
  double legWeights[nLegs];
  for (int i=0;i<nLegs;i++) {
    legWeights[i] = 1.0;
  }

  double sumWeights = 0;
  int nStanceLegs = 0;
  int iLeg=0;
  for ( auto leg : *legs_) {
    //if (leg->isInStanceMode()) {
    if (leg->isSupportLeg()) {
      double t = 1 - mapTo01Range(leg->getStancePhase(),startShiftAwayFromLegAtStancePhase_, 1.0);
      t = linearlyInterpolate(minSwingLegWeight_, 1, 0, 1, t);
      legWeights[iLeg] = t;
      nStanceLegs++;
    } else {
      double t = mapTo01Range(leg->getSwingPhase(), startShiftTowardsLegAtSwingPhase_, 1.0);
      t = linearlyInterpolate(minSwingLegWeight_, 1, 0, 1, t);
      legWeights[iLeg] = t;
    }
    sumWeights += legWeights[iLeg];
    iLeg++;
  }

  Position comTarget;

  iLeg=0;
  for (auto leg : *legs_) {
    //	            tprintf("leg %d(%s): stanceMode: %lf, swingMode: %lf. Weight:%lf\n", i, leg->getName(), leg->getStancePhase(),leg->getSwingPhase(), legWeights[i]);
    const Position positionWorldToFootInWorldFrame = leg->getWorldToFootPositionInWorldFrame();

    if (sumWeights != 0) {
      comTarget += Position(positionWorldToFootInWorldFrame.toImplementation()*legWeights[iLeg]/sumWeights);
    }
    iLeg++;
  }

  desiredWorldToFootPositionInWorldFrame_ = comTarget+Position(headingOffset_, lateralOffset_, 0.0);

//  std::cout << "desired world to foot pos: " << desiredWorldToFootPositionInWorldFrame_ << std::endl;

}

bool CoMOverSupportPolygonControl::loadParameters(TiXmlHandle &hParameterSet) {

	TiXmlElement* pElem;
	TiXmlHandle handle(hParameterSet.FirstChild("CoMOverSupportPolygonControl"));



	pElem = handle.FirstChild("Weight").Element();
	if (!pElem) {
		printf("Could not find CoMOverSupportPolygonControl:Weight\n");
		return false;
	}

	if (pElem->QueryDoubleAttribute("minSwingLegWeight", &this->minSwingLegWeight_)
			!= TIXML_SUCCESS) {
		printf("Could not find CoMOverSupportPolygonControl:Weight:minSwingLegWeight!\n");
		return false;
	}

  pElem = handle.FirstChild("Timing").Element();
	if (pElem->QueryDoubleAttribute("startShiftAwayFromLegAtStancePhase",
			&this->startShiftAwayFromLegAtStancePhase_) != TIXML_SUCCESS) {
		printf("Could not find CoMOverSupportPolygonControl:Timing:startShiftAwayFromLegAtStancePhase!\n");
		return false;
	}

	if (pElem->QueryDoubleAttribute("startShiftTowardsLegAtSwingPhase",
			&this->startShiftTowardsLegAtSwingPhase_) != TIXML_SUCCESS) {
		printf("Could not find SupportPolygon:startShiftTowardsLegAtSwingPhase!\n");
		return false;
	}
//	pElem = hParameterSet.FirstChild("SupportPolygon").FirstChild("Offset").Element();
//	if (pElem) {
//		if (pElem->QueryDoubleAttribute("sagittalOffset",
//				&this->sagittalOffset) != TIXML_SUCCESS) {
//			sagittalOffset = 0.0;
//		}
//
//		if (pElem->QueryDoubleAttribute("coronalOffset",
//				&this->coronalOffset) != TIXML_SUCCESS) {
//			coronalOffset = 0.0;
//		}
//
//	}

	return true;

}

bool CoMOverSupportPolygonControl::saveParameters(TiXmlHandle &hParameterSet) {

//	double value;
//	TiXmlElement* pElem;
//
//	pElem = hParameterSet.FirstChild("SupportPolygon").Element();
//	if (!pElem) {
//		printf("Could not find SupportPolygon\n");
//		return false;
//	}
//	pElem->SetDoubleAttribute("minLegWeight", this->spMinLegWeight);
//	pElem->SetDoubleAttribute("stPShiftAwayFromLegStart",
//			this->spStPShiftAwayFromLegStart);
//	pElem->SetDoubleAttribute("swPShiftTowardsLegStart",
//			this->spSwPShiftTowardsLegStart);

	return false;

}

bool CoMOverSupportPolygonControl::setToInterpolated(const CoMOverSupportPolygonControl& supportPolygon1,
		const CoMOverSupportPolygonControl& supportPolygon2, double t) {
	this->minSwingLegWeight_ = linearlyInterpolate(supportPolygon1.minSwingLegWeight_,
			supportPolygon2.minSwingLegWeight_, 0, 1, t);
	this->startShiftAwayFromLegAtStancePhase_ = linearlyInterpolate(
			supportPolygon1.startShiftAwayFromLegAtStancePhase_,
			supportPolygon2.startShiftAwayFromLegAtStancePhase_, 0, 1, t);
	this->startShiftTowardsLegAtSwingPhase_ = linearlyInterpolate(
			supportPolygon1.startShiftTowardsLegAtSwingPhase_,
			supportPolygon2.startShiftTowardsLegAtSwingPhase_, 0, 1, t);

	this->headingOffset_ = linearlyInterpolate(
			supportPolygon1.headingOffset_,
			supportPolygon2.headingOffset_, 0, 1, t);

	this->lateralOffset_ = linearlyInterpolate(
			supportPolygon1.lateralOffset_,
			supportPolygon2.lateralOffset_, 0, 1, t);

	return true;
}



const Position& CoMOverSupportPolygonControl::getDesiredWorldToCoMPositionInWorldFrame() const {
  return desiredWorldToFootPositionInWorldFrame_;
}


} // namespace loco
