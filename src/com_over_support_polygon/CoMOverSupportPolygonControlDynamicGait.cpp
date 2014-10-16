/*!
 * @file 	    CoMOverSupportPolygonControl.cpp
 * @author 	  Christian Gehring, Stelian Coros
 * @date		  Jul 17, 2012
 * @version 	1.0
 * @ingroup
 * @brief
 */

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlDynamicGait.hpp"
#include "loco/common/TerrainModelFreePlane.hpp"
#include "loco/temp_helpers/math.hpp"

using namespace std;

namespace loco {


CoMOverSupportPolygonControlDynamicGait::CoMOverSupportPolygonControlDynamicGait(LegGroup* legs) :
    CoMOverSupportPolygonControlBase(legs)
{

}


CoMOverSupportPolygonControlDynamicGait::~CoMOverSupportPolygonControlDynamicGait() {

}


void CoMOverSupportPolygonControlDynamicGait::advance(double dt) {

  const int nLegs = legs_->size();
  double legWeights[nLegs];
  for (int i=0;i<nLegs;i++) {
    legWeights[i] = 1.0;
  }

  double sumWeights = 0;
  int iLeg=0;
  for ( auto leg : *legs_) {
    //if (leg->isInStanceMode()) {
    //if (leg->isSupportLeg(  )) {
    if ( (leg->isSupportLeg() && (leg->getSwingPhase() > 0.8)) || leg->shouldBeGrounded() ) { // correct
//    if ( false ) {
      double t = 1 - mapTo01Range(leg->getStancePhase(),startShiftAwayFromLegAtStancePhase_, 1.0);
      t = linearlyInterpolate(minSwingLegWeight_, 1, 0, 1, t);
      legWeights[iLeg] = t;
    } else {
      double t = mapTo01Range(leg->getSwingPhase(), startShiftTowardsLegAtSwingPhase_, 1.0);
      t = linearlyInterpolate(minSwingLegWeight_, 1, 0, 1, t);
      legWeights[iLeg] = t;
    }
    sumWeights += legWeights[iLeg];
    iLeg++;
  }

  Position comTarget;

  if (sumWeights != 0) {
    iLeg=0;
    for (auto leg : *legs_) {
      //	            tprintf("leg %d(%s): stanceMode: %lf, swingMode: %lf. Weight:%lf\n", i, leg->getName(), leg->getStancePhase(),leg->getSwingPhase(), legWeights[i]);
      comTarget += leg->getPositionWorldToFootInWorldFrame()*legWeights[iLeg];
      iLeg++;
    }
    comTarget /= sumWeights;
  } else {
    for (auto leg : *legs_) {
      comTarget += leg->getPositionWorldToFootInWorldFrame();
    }
    comTarget /= legs_->size();
  }

  positionWorldToDesiredCoMInWorldFrame_ = comTarget + Position(headingOffset_, lateralOffset_, 0.0);
  positionWorldToDesiredCoMInWorldFrame_.z() = 0.0;

//  std::cout << "desired world to foot pos: " << positionWorldToHorizontalDesiredBaseInWorldFrame_ << std::endl;

}


const Position& CoMOverSupportPolygonControlDynamicGait::getPositionWorldToDesiredCoMInWorldFrame() const {
  return positionWorldToDesiredCoMInWorldFrame_;
}


bool CoMOverSupportPolygonControlDynamicGait::setToInterpolated(const CoMOverSupportPolygonControlBase& supportPolygon1,
                                                                const CoMOverSupportPolygonControlBase& supportPolygon2, double t) {

  const CoMOverSupportPolygonControlDynamicGait& supportPolygon1_this = static_cast<const CoMOverSupportPolygonControlDynamicGait&>(supportPolygon1);
  const CoMOverSupportPolygonControlDynamicGait& supportPolygon2_this = static_cast<const CoMOverSupportPolygonControlDynamicGait&>(supportPolygon2);

  this->minSwingLegWeight_ = linearlyInterpolate(supportPolygon1.getMinSwingLegWeight(),
      supportPolygon2_this.getMinSwingLegWeight(), 0, 1, t);
  this->startShiftAwayFromLegAtStancePhase_ = linearlyInterpolate(
      supportPolygon1_this.getStartShiftAwayFromLegAtStancePhase(),
      supportPolygon2_this.getStartShiftAwayFromLegAtStancePhase(), 0, 1, t);
  this->startShiftTowardsLegAtSwingPhase_ = linearlyInterpolate(
      supportPolygon1_this.getStartShiftTowardsLegAtSwingPhase(),
      supportPolygon2_this.getStartShiftTowardsLegAtSwingPhase(), 0, 1, t);

  this->headingOffset_ = linearlyInterpolate(
      supportPolygon1_this.getHeadingOffset(),
      supportPolygon2_this.getHeadingOffset(), 0, 1, t);

  this->lateralOffset_ = linearlyInterpolate(
      supportPolygon1_this.getLateralOffset(),
      supportPolygon2_this.getLateralOffset(), 0, 1, t);

  return true;
}


} // namespace loco
