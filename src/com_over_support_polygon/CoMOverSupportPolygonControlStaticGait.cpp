/*
 * CoMOverSupportPolygonStaticGait.cpp
 *
 *  Created on: Oct 7, 2014
 *      Author: dario
 */

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlStaticGait.hpp"

namespace loco {

CoMOverSupportPolygonControlStaticGait::CoMOverSupportPolygonControlStaticGait(LegGroup *legs):
    CoMOverSupportPolygonControl(legs),
    swingOrder_(legs_->size()),
    positionWorldToHorizontalDesiredBaseInWorldFrame_()
{

}


CoMOverSupportPolygonControlStaticGait::~CoMOverSupportPolygonControlStaticGait() {

}


void CoMOverSupportPolygonControlStaticGait::advance(double dt) {

}


Eigen::Matrix<double,3,4> CoMOverSupportPolygonControlStaticGait::getNextStanceConfig(Eigen::Matrix<double,3,4> currentStanceConfig, int steppingFoot) {
  Eigen::Matrix<double,3,4> mat;
  mat.setZero();
  return mat;
}


int CoMOverSupportPolygonControlStaticGait::getNextSwingFoot(int currentSwingFoot) {
  return 0;
}


Position CoMOverSupportPolygonControlStaticGait::lineIntersect(Eigen::Matrix<double,3,2> l1, Eigen::Matrix<double,3,2> l2) {
  return Position();
}


const Position& CoMOverSupportPolygonControlStaticGait::getDesiredWorldToCoMPositionInWorldFrame() const {
  return positionWorldToHorizontalDesiredBaseInWorldFrame_;
}

} /* namespace loco */
