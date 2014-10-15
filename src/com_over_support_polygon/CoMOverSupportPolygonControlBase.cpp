/*
 * CoMOverSupportPolygonControlBase.cpp
 *
 *  Created on: Oct 7, 2014
 *      Author: dario
 */

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlBase.hpp"
#include <stdio.h>
#include <Eigen/Dense>
#include "loco/temp_helpers/math.hpp"

namespace loco {

CoMOverSupportPolygonControlBase::CoMOverSupportPolygonControlBase(LegGroup* legs):
  legs_(legs){
  // trot
//    minSwingLegWeight_ = 0.15;
//    startShiftAwayFromLegAtStancePhase_ = 0.7;
//    startShiftTowardsLegAtSwingPhase_ = 0.7;
//    headingOffset_ = 0;
//    lateralOffset_ = 0;

  minSwingLegWeight_ = 0.0;
  startShiftAwayFromLegAtStancePhase_ = 0.0;
  startShiftTowardsLegAtSwingPhase_ = 0.0;
  headingOffset_ = 0.0;
  lateralOffset_ = 0.0;

}


CoMOverSupportPolygonControlBase::~CoMOverSupportPolygonControlBase() {

}


bool CoMOverSupportPolygonControlBase::loadParameters(TiXmlHandle &hParameterSet) {

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
//  pElem = hParameterSet.FirstChild("SupportPolygon").FirstChild("Offset").Element();
//  if (pElem) {
//    if (pElem->QueryDoubleAttribute("sagittalOffset",
//        &this->sagittalOffset) != TIXML_SUCCESS) {
//      sagittalOffset = 0.0;
//    }
//
//    if (pElem->QueryDoubleAttribute("coronalOffset",
//        &this->coronalOffset) != TIXML_SUCCESS) {
//      coronalOffset = 0.0;
//    }
//
//  }

  return true;

}

bool CoMOverSupportPolygonControlBase::saveParameters(TiXmlHandle &hParameterSet) {

//  double value;
//  TiXmlElement* pElem;
//
//  pElem = hParameterSet.FirstChild("SupportPolygon").Element();
//  if (!pElem) {
//    printf("Could not find SupportPolygon\n");
//    return false;
//  }
//  pElem->SetDoubleAttribute("minLegWeight", this->spMinLegWeight);
//  pElem->SetDoubleAttribute("stPShiftAwayFromLegStart",
//      this->spStPShiftAwayFromLegStart);
//  pElem->SetDoubleAttribute("swPShiftTowardsLegStart",
//      this->spSwPShiftTowardsLegStart);

  return false;

}

double CoMOverSupportPolygonControlBase::getMinSwingLegWeight() const {
  return minSwingLegWeight_;
}
double CoMOverSupportPolygonControlBase::getStartShiftAwayFromLegAtStancePhase() const {
  return startShiftAwayFromLegAtStancePhase_;
}
double CoMOverSupportPolygonControlBase::getStartShiftTowardsLegAtSwingPhase() const {
  return startShiftTowardsLegAtSwingPhase_;
}
double CoMOverSupportPolygonControlBase::getLateralOffset() const {
  return lateralOffset_;
}
double CoMOverSupportPolygonControlBase::getHeadingOffset() const {
  return headingOffset_;
}


//bool CoMOverSupportPolygonControlBase::setToInterpolated(const CoMOverSupportPolygonControlBase& supportPolygon1,
//    const CoMOverSupportPolygonControlBase& supportPolygon2, double t) {
//  this->minSwingLegWeight_ = linearlyInterpolate(supportPolygon1.minSwingLegWeight_,
//      supportPolygon2.minSwingLegWeight_, 0, 1, t);
//  this->startShiftAwayFromLegAtStancePhase_ = linearlyInterpolate(
//      supportPolygon1.startShiftAwayFromLegAtStancePhase_,
//      supportPolygon2.startShiftAwayFromLegAtStancePhase_, 0, 1, t);
//  this->startShiftTowardsLegAtSwingPhase_ = linearlyInterpolate(
//      supportPolygon1.startShiftTowardsLegAtSwingPhase_,
//      supportPolygon2.startShiftTowardsLegAtSwingPhase_, 0, 1, t);
//
//  this->headingOffset_ = linearlyInterpolate(
//      supportPolygon1.headingOffset_,
//      supportPolygon2.headingOffset_, 0, 1, t);
//
//  this->lateralOffset_ = linearlyInterpolate(
//      supportPolygon1.lateralOffset_,
//      supportPolygon2.lateralOffset_, 0, 1, t);
//
//  return true;
//}

} /* namespace loco */
