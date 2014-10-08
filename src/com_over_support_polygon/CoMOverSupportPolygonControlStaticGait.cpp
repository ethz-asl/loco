/*
 * CoMOverSupportPolygonStaticGait.cpp
 *
 *  Created on: Oct 7, 2014
 *      Author: dario
 */

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlStaticGait.hpp"
#include <Eigen/LU>

namespace loco {

CoMOverSupportPolygonControlStaticGait::CoMOverSupportPolygonControlStaticGait(LegGroup *legs):
    CoMOverSupportPolygonControlBase(legs),
    swingOrder_(legs_->size()),
    positionWorldToHorizontalDesiredBaseInWorldFrame_()
{
  homePos_.setZero();
}


CoMOverSupportPolygonControlStaticGait::~CoMOverSupportPolygonControlStaticGait() {

}


void CoMOverSupportPolygonControlStaticGait::advance(double dt) {
  //  % cFPl = matrix with all foot points at landing
  //  % fnrl = foot number of foot that just landed
  //  % rmbl = main body position at landing
  //
  //  % using the feet position and swing leg number, we update the 3
  //  % steps (last, next, and overnext)
  //  obj.swing0 = fnrl;
  //  obj.swing1 = obj.getNextSwingFoot(obj.swing0);
  //  obj.swing2 = obj.getNextSwingFoot(obj.swing1);
  //
  //  % update the foot points at landing
  //  obj.cFP1 = cFPl;
  //  % after one step, we would be at the current position
  //  obj.cFP2 = obj.getNextStanceConfig(cFPl,obj.swing1);


    getIndexOfSwingLeg();
}


bool CoMOverSupportPolygonControlStaticGait::initialize() {
  positionWorldToHorizontalDesiredBaseInWorldFrame_ = Position(0.0,0.0,0.0);
  return true;
}


Eigen::Matrix<double,3,4> CoMOverSupportPolygonControlStaticGait::getNextStanceConfig(Eigen::Matrix<double,3,4> currentStanceConfig, int steppingFoot) {
  Eigen::Matrix<double,3,4> mat;
  mat.setZero();
  return mat;
}


int CoMOverSupportPolygonControlStaticGait::getNextSwingFoot(int currentSwingFoot) {
  return 0;
}


Position CoMOverSupportPolygonControlStaticGait::lineIntersect(Eigen::Matrix<double,2,2> l1, Eigen::Matrix<double,2,2> l2) {

  Eigen::Vector2d l1_1 = l1.col(0);
  Eigen::Vector2d l1_2 = l1.col(1);

  Eigen::Vector2d l2_1 = l2.col(0);
  Eigen::Vector2d l2_2 = l2.col(1);

  Eigen::Vector2d v1 = l1_2-l1_1;
  v1 = v1/v1.norm();

  Eigen::Vector2d v2 = l1_2-l1_1;
  v2 = v2/v2.norm();

  Eigen::Matrix2d A;
  A << -v1, v2;
  Eigen::Vector2d x = A.lu().solve(l1_1-l2_1);

  Position intersection = Position(l1_1(0) + x(0)*v1(0),
                                   l1_1(1) + x(1)*v1(1),
                                   0.0);

  return intersection;
}


const Position& CoMOverSupportPolygonControlStaticGait::getDesiredWorldToCoMPositionInWorldFrame() const {
  return positionWorldToHorizontalDesiredBaseInWorldFrame_;
}


int CoMOverSupportPolygonControlStaticGait::getIndexOfSwingLeg() {
  int swingLeg = -1;
  for (auto leg: *legs_) {
    if (leg->getSwingPhase() != -1) {
      std::cout << "swing leg nr: " << leg->getId() << std::endl;
      swingLeg = leg->getId();
    }
  }
  return swingLeg;
}


} /* namespace loco */
