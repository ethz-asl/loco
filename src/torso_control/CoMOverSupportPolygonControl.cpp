/*!
 * @file 	    CoMOverSupportPolygonControl.cpp
 * @author 	  Christian Gehring, Stelian Coros
 * @date		  Jul 17, 2012
 * @version 	1.0
 * @ingroup
 * @brief
 */

#include "loco/torso_control/CoMOverSupportPolygonControl.hpp"
#include "loco/common/TerrainModelFreePlane.hpp"
#include <stdio.h>
#include <Eigen/Dense>
#include "loco/temp_helpers/math.hpp"

using namespace std;

namespace loco {


CoMOverSupportPolygonControl::CoMOverSupportPolygonControl(LegGroup* legs) :
legs_(legs),
torso_(),
terrainModel_()
{

	// trot
	minSwingLegWeight_ = 0.15;
	startShiftAwayFromLegAtStancePhase_ = 0.7;
	startShiftTowardsLegAtSwingPhase_ = 0.7;
	headingOffset_ = 0;
	lateralOffset_ = 0;


	positionCenterToForeHindSupportFeetInControlFrame_[0] = Position::Zero();
	positionCenterToForeHindSupportFeetInControlFrame_[1] = Position::Zero();
	positionWorldToCenterInWorldFrame_ = Position::Zero();

}


CoMOverSupportPolygonControl::~CoMOverSupportPolygonControl() {

}


void CoMOverSupportPolygonControl::setTorso(TorsoBase* torso) { torso_ = torso; }
void CoMOverSupportPolygonControl::setTerrainModel(TerrainModelBase* terrainModel) { terrainModel_ = terrainModel;}


void CoMOverSupportPolygonControl::advance(double dt) {
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
      comTarget += leg->getWorldToFootPositionInWorldFrame()*legWeights[iLeg];
      iLeg++;
    }
    comTarget /= sumWeights;
  } else {
    for (auto leg : *legs_) {
      comTarget += leg->getWorldToFootPositionInWorldFrame();
    }
    comTarget /= legs_->size();
  }

  positionWorldToHorizontalDesiredBaseInWorldFrame_ = comTarget;//; + Position(headingOffset_, lateralOffset_, 0.0);
  positionWorldToHorizontalDesiredBaseInWorldFrame_.z() = 0.0;

//  std::cout << "desired world to foot pos: " << positionWorldToHorizontalDesiredBaseInWorldFrame_ << std::endl;

}

void CoMOverSupportPolygonControl::advanceLeverConfiguration(double dt) {

  Position comTarget;

  // Matlab code
//  % Find ratios - control vertical
//  ratio_x_controlvertical = abs((rb_onplane_worldvertical(1) - rf_lf_controlvertical(1))/...
//                            (rb_onplane_worldvertical(1) - rf_lh_controlvertical(1)));
//  ratio_y_controlvertical = abs((rb_onplane_worldvertical(2) - rf_lf_controlvertical(2))/...
//                            (rb_onplane_worldvertical(2) - rf_rf_controlvertical(2)));
//
//  shift_y = hipToHipY*ratio_x_controlvertical/(1+ratio_x_controlvertical) - rf_lf_controlvertical(2);

  RotationQuaternion orientationWorldToBase = torso_->getMeasuredState().getOrientationWorldToBase();
  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();

  TerrainModelFreePlane* terrainModel = (TerrainModelFreePlane*)terrainModel_;

  Position rb_onplane_worldvertical = torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
  terrainModel->getHeight(rb_onplane_worldvertical);

  Position rb_onplane_controlvertical = terrainModel->getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame());
//  std::cout << "rb onplane control: " << rb_onplane_controlvertical << std::endl;
//  std::cout << "rb onplane world: " << rb_onplane_worldvertical << std::endl;

  positionWorldToCenterInWorldFrame_ = rb_onplane_controlvertical;

  Position positionBaseOnTerrainToForeSupportLegInControlFrame;
  Position positionBaseOnTerrainToHindSupportLegInControlFrame;

  // Get contact points
  if (legs_->getLeftForeLeg()->isSupportLeg()) {
    positionBaseOnTerrainToForeSupportLegInControlFrame = terrainModel->getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(legs_->getLeftForeLeg()->getWorldToFootPositionInWorldFrame());
    positionBaseOnTerrainToForeSupportLegInControlFrame = orientationWorldToControl.rotate(positionBaseOnTerrainToForeSupportLegInControlFrame-rb_onplane_controlvertical);

    positionBaseOnTerrainToHindSupportLegInControlFrame = terrainModel->getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(legs_->getRightHindLeg()->getWorldToFootPositionInWorldFrame());
    positionBaseOnTerrainToHindSupportLegInControlFrame = orientationWorldToControl.rotate(positionBaseOnTerrainToHindSupportLegInControlFrame-rb_onplane_controlvertical);

    positionCenterToForeHindSupportFeetInControlFrame_[0] = positionBaseOnTerrainToForeSupportLegInControlFrame;
    positionCenterToForeHindSupportFeetInControlFrame_[1] = positionBaseOnTerrainToHindSupportLegInControlFrame;
  }
  else {
    positionBaseOnTerrainToForeSupportLegInControlFrame = terrainModel->getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(legs_->getRightForeLeg()->getWorldToFootPositionInWorldFrame());
    positionBaseOnTerrainToForeSupportLegInControlFrame = orientationWorldToControl.rotate(positionBaseOnTerrainToForeSupportLegInControlFrame-rb_onplane_controlvertical);

    positionBaseOnTerrainToHindSupportLegInControlFrame = terrainModel->getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(legs_->getLeftHindLeg()->getWorldToFootPositionInWorldFrame());
    positionBaseOnTerrainToHindSupportLegInControlFrame = orientationWorldToControl.rotate(legs_->getLeftHindLeg()->getWorldToFootPositionInWorldFrame()-rb_onplane_controlvertical);

    positionCenterToForeHindSupportFeetInControlFrame_[0] = positionBaseOnTerrainToForeSupportLegInControlFrame;
    positionCenterToForeHindSupportFeetInControlFrame_[1] = positionBaseOnTerrainToHindSupportLegInControlFrame;

  }

  Position r_controlToBaseOnTerrainInControlFrame = orientationWorldToControl.rotate(rb_onplane_worldvertical - rb_onplane_controlvertical);

  double ratioX = fabs(r_controlToBaseOnTerrainInControlFrame.x() - positionBaseOnTerrainToForeSupportLegInControlFrame.x())
                  /
                  fabs(r_controlToBaseOnTerrainInControlFrame.x() - positionBaseOnTerrainToHindSupportLegInControlFrame.x());

  double hipToHipX = fabs(positionBaseOnTerrainToForeSupportLegInControlFrame.x() - positionBaseOnTerrainToHindSupportLegInControlFrame.x());
  double hipToHipY = fabs(positionBaseOnTerrainToForeSupportLegInControlFrame.y() - positionBaseOnTerrainToHindSupportLegInControlFrame.y());

  comTarget = r_controlToBaseOnTerrainInControlFrame;

  double ly_2 = 0.0;
  double ratioY = 0.0;

  if (legs_->getLeftForeLeg()->isSupportLeg()) {
    ly_2 = hipToHipY/(1.0+ratioX);
    comTarget.y() = - (hipToHipY*0.5 - ly_2);
  }
  else {
    ly_2 = hipToHipY*ratioX/(1.0+ratioX);
    comTarget.y() = ly_2 - hipToHipY*0.5;
  }

  Position aux = rb_onplane_controlvertical + orientationWorldToControl.inverseRotate(comTarget);
  comTarget = aux;

  ratioY = fabs(comTarget.y() - positionBaseOnTerrainToForeSupportLegInControlFrame.y())
           /
           fabs(comTarget.y() - positionBaseOnTerrainToHindSupportLegInControlFrame.y());

//  std::cout << "check ratios" << std::endl
//            << "x: " << ratioX << std::endl
//            << "y: " << ratioY << std::endl
//            << "ly2/ly1: " << ly_2/(hipToHipY-ly_2) << std::endl
//            << "ly1/ly2: " << 1.0/(ly_2/(hipToHipY-ly_2)) << std::endl
//            << "com: " << comTarget << std::endl
//            << "fore support" << positionBaseOnTerrainToForeSupportLegInControlFrame << std::endl;

  positionWorldToHorizontalDesiredBaseInWorldFrame_ = comTarget + Position(headingOffset_, lateralOffset_, 0.0);
  positionWorldToHorizontalDesiredBaseInWorldFrame_.z() = 0.0;

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
  return positionWorldToHorizontalDesiredBaseInWorldFrame_;
}


} // namespace loco
