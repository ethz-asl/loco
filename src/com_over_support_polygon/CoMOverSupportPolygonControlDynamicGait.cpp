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
	// trot
	minSwingLegWeight_ = 0.15;
	startShiftAwayFromLegAtStancePhase_ = 0.7;
	startShiftTowardsLegAtSwingPhase_ = 0.7;
	headingOffset_ = 0;
	lateralOffset_ = 0;
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

  positionWorldToHorizontalDesiredBaseInWorldFrame_ = comTarget;//; + Position(headingOffset_, lateralOffset_, 0.0);
  positionWorldToHorizontalDesiredBaseInWorldFrame_.z() = 0.0;

//  std::cout << "desired world to foot pos: " << positionWorldToHorizontalDesiredBaseInWorldFrame_ << std::endl;

}


const Position& CoMOverSupportPolygonControlDynamicGait::getPositionWorldToDesiredCoMInWorldFrame() const {
  return positionWorldToHorizontalDesiredBaseInWorldFrame_;
}


} // namespace loco
