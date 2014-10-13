/*
 * CoMOverSupportPolygonStaticGait.cpp
 *
 *  Created on: Oct 7, 2014
 *      Author: dario
 */

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlStaticGait.hpp"
#include <Eigen/LU>
//#include <algorithm>

namespace loco {

CoMOverSupportPolygonControlStaticGait::CoMOverSupportPolygonControlStaticGait(LegGroup *legs, TorsoBase* torso):
    CoMOverSupportPolygonControlBase(legs),
    torso_(torso),
    swingOrder_(legs_->size()),
    delta_(0.0),
    maxComStep_(0.0),
    swingLegIndexNow_(-1),
    swingLegIndexNext_(-1),
    swingLegIndexLast_(-1),
    swingLegIndexOverNext_(-1),
    swingFootChanged_(false),
    filterInputCoMX_(0),
    filterInputCoMY_(0),
    filterInputCoMZ_(0),
    filterOutputCoMX_(0),
    filterOutputCoMY_(0),
    filterOutputCoMZ_(0),
    makeShift_(false)
{
  // Reset Eigen variables
  homePos_.setZero();
  comStep_.setZero();

  feetConfigurationCurrent_.setZero();
  feetConfigurationNext_.setZero();
  feetConfigurationOverNext_.setZero();

  supportTriangleCurrent_.setZero();
  supportTriangleNext_.setZero();
  supportTriangleOverNext_.setZero();

  safeTriangleCurrent_.setZero();
  safeTriangleNext_.setZero();
  safeTriangleOverNext_.setZero();

  maxComStep_ = 0.5;
  delta_ = 0.04;

  comTarget_.setZero();

  filterCoMX_ = new robotUtils::FirstOrderFilter();
  filterCoMY_ = new robotUtils::FirstOrderFilter();
  filterCoMZ_ = new robotUtils::FirstOrderFilter();

}


CoMOverSupportPolygonControlStaticGait::~CoMOverSupportPolygonControlStaticGait() {
  delete filterCoMX_;
  delete filterCoMY_;
  delete filterCoMZ_;
}


bool CoMOverSupportPolygonControlStaticGait::initialize() {

  makeShift_ = false;

  comTarget_.setZero();

  double filterTimeConstant = 0.0005;
  double filterGain = 1.0;

  filterCoMX_->initialize(filterInputCoMX_, filterTimeConstant, filterGain);
  filterCoMY_->initialize(filterInputCoMY_, filterTimeConstant, filterGain);
  filterCoMZ_->initialize(filterInputCoMZ_, filterTimeConstant, filterGain);

  maxComStep_ = 0.5;
  delta_ = 0;//0.04;

  positionWorldToDesiredCoMInWorldFrame_ = torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
  positionWorldToDesiredCoMInWorldFrame_.z() = 0.0;

  feetConfigurationCurrent_.setZero();
  feetConfigurationNext_.setZero();
  feetConfigurationOverNext_.setZero();

  // Leg swing order
  swingOrder_[0] = 0;
  swingOrder_[1] = 3;
  swingOrder_[2] = 1;
  swingOrder_[3] = 2;

  swingLegIndexLast_ = swingOrder_[0];
  swingLegIndexNow_ = swingLegIndexLast_;
  swingLegIndexNext_ = getNextSwingFoot(swingLegIndexLast_);
  swingLegIndexOverNext_= getNextSwingFoot(swingLegIndexNext_);

  // save home feet positions
  for (auto leg: *legs_) {
    Position positionWorldToFootInFrame = leg->getPositionWorldToFootInWorldFrame();
    homePos_.col(leg->getId()) << positionWorldToFootInFrame.x(), positionWorldToFootInFrame.y();
  }

  feetConfigurationCurrent_ = homePos_;
  feetConfigurationNext_ = getNextStanceConfig(feetConfigurationCurrent_, swingLegIndexNext_);
  feetConfigurationOverNext_ = getNextStanceConfig(feetConfigurationNext_, swingLegIndexOverNext_);

  updateSafeSupportTriangles();

  return true;
}


const Position& CoMOverSupportPolygonControlStaticGait::getPositionWorldToDesiredCoMInWorldFrame() const {
  return positionWorldToDesiredCoMInWorldFrame_;
}


void CoMOverSupportPolygonControlStaticGait::updateSafeSupportTriangles() {
  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  LinearVelocity desiredLinearVelocityInWorldFrame = orientationWorldToControl.inverseRotate(torso_->getDesiredState().getLinearVelocityBaseInControlFrame());
  Eigen::Vector2d desiredLinearVelocity;
  desiredLinearVelocity << desiredLinearVelocityInWorldFrame.x(),
                           desiredLinearVelocityInWorldFrame.y();
  comStep_ = desiredLinearVelocity*legs_->getLeftForeLeg()->getStanceDuration();

  // update current configuration
  for (auto leg: *legs_) {
    Position positionWorldToFootInWorldFrame = leg->getPositionWorldToFootInWorldFrame();
    feetConfigurationCurrent_.col(leg->getId()) << positionWorldToFootInWorldFrame.x(), positionWorldToFootInWorldFrame.y();
  }

  feetConfigurationNext_ = getNextStanceConfig(feetConfigurationCurrent_, swingLegIndexLast_);
  feetConfigurationOverNext_ = getNextStanceConfig(feetConfigurationNext_, swingLegIndexNext_);

  // Get support triangles
  int j=0;
  for (int k=0; k<legs_->size(); k++) {
    if (k != swingLegIndexLast_) {
      supportTriangleCurrent_.col(j) = feetConfigurationCurrent_.col(k);
      j++;
    }
  }
  safeTriangleCurrent_ = getSafeTriangle(supportTriangleCurrent_);

  j=0;
  for (int k=0; k<legs_->size(); k++) {
    if (k != swingLegIndexNext_) {
      supportTriangleNext_.col(j) = feetConfigurationNext_.col(k);
      j++;
    }
  }
  safeTriangleNext_ = getSafeTriangle(supportTriangleNext_);

  j=0;
  for (int k=0; k<legs_->size(); k++) {
    if (k != swingLegIndexOverNext_) {
      supportTriangleOverNext_.col(j) = feetConfigurationOverNext_.col(k);
      j++;
    }
  }
  safeTriangleOverNext_ = getSafeTriangle(supportTriangleOverNext_);


  std::vector<int> diagonalSwingLegsLast(2), diagonalSwingLegsNext(2), diagonalSwingLegsOverNext(2);
  diagonalSwingLegsLast = getDiagonalElements(swingLegIndexLast_);
  diagonalSwingLegsNext = getDiagonalElements(swingLegIndexNext_);
  diagonalSwingLegsOverNext = getDiagonalElements(swingLegIndexOverNext_);

  Pos2d intersection;
  intersection.setZero();

  Line lineSafeLast, lineSafeNext, lineSafeOverNext;
  lineSafeLast << safeTriangleCurrent_.col(diagonalSwingLegsLast[0]),
                  safeTriangleCurrent_.col(diagonalSwingLegsLast[1]);

  lineSafeNext << safeTriangleNext_.col(diagonalSwingLegsNext[0]),
                  safeTriangleNext_.col(diagonalSwingLegsNext[1]);

  lineSafeOverNext << safeTriangleOverNext_.col(diagonalSwingLegsOverNext[0]),
                      safeTriangleOverNext_.col(diagonalSwingLegsOverNext[1]);

  if (lineIntersect(lineSafeLast, lineSafeNext, intersection)) {
    comTarget_ = intersection;
    makeShift_ = false;
  }
  else if (lineIntersect(lineSafeNext, lineSafeOverNext, intersection)) {
    comTarget_ = intersection;
    makeShift_ = true;
  }
//  else if (lineIntersect(lineSafeLast, lineSafeOverNext, intersection)) {
//        positionWorldToDesiredCoMInWorldFrame_ << intersection[0],
//                                                  intersection[1],
//                                                  0.0;
//  }
  else {
    comTarget_ = (safeTriangleCurrent_.col(0)+safeTriangleCurrent_.col(1)+safeTriangleCurrent_.col(2))/3.0;
  }

}


bool CoMOverSupportPolygonControlStaticGait::setToInterpolated(const CoMOverSupportPolygonControlBase& supportPolygon1, const CoMOverSupportPolygonControlBase& supportPolygon2, double t) {
  return false;
}


void CoMOverSupportPolygonControlStaticGait::advance(double dt) {
    updateSwingLegsIndexes();

    if (swingFootChanged_) {
      // reset flag
      swingFootChanged_ = false;
      updateSafeSupportTriangles();
    }

    filterInputCoMX_ = comTarget_.x();
    filterInputCoMY_ = comTarget_.y();

    filterOutputCoMX_ = filterCoMX_->advance(dt,filterInputCoMX_);
    filterOutputCoMY_ = filterCoMY_->advance(dt,filterInputCoMY_);

    if (1+0*makeShift_) {
      positionWorldToDesiredCoMInWorldFrame_ = Position(filterOutputCoMX_,
                                                        filterOutputCoMY_,
                                                        0.0);
    }
}


Eigen::Matrix<double,2,3> CoMOverSupportPolygonControlStaticGait::getSupportTriangleCurrent() const {
  return supportTriangleCurrent_;
}


Eigen::Matrix<double,2,3> CoMOverSupportPolygonControlStaticGait::getSupportTriangleNext() const {
  return supportTriangleNext_;
}


Eigen::Matrix<double,2,3> CoMOverSupportPolygonControlStaticGait::getSupportTriangleOverNext() const {
  return supportTriangleOverNext_;
}

Eigen::Matrix<double,2,3> CoMOverSupportPolygonControlStaticGait::getSafeTriangleCurrent() const {
  return safeTriangleCurrent_;
}

Eigen::Matrix<double,2,3> CoMOverSupportPolygonControlStaticGait::getSafeTriangleNext() const {
  return safeTriangleNext_;
}

Eigen::Matrix<double,2,3> CoMOverSupportPolygonControlStaticGait::getSafeTriangleOverNext() const {
  return safeTriangleOverNext_;
}


Eigen::Matrix<double,2,4> CoMOverSupportPolygonControlStaticGait::getNextStanceConfig(const FeetConfiguration& currentStanceConfig, int steppingFoot) {

  FeetConfiguration nextStanceConfig = currentStanceConfig;
  nextStanceConfig.col(steppingFoot) += comStep_;

  return nextStanceConfig;
}


bool CoMOverSupportPolygonControlStaticGait::lineIntersect(const Line& l1, const Line& l2, Pos2d& intersection) {
  Pos2d l1_1 = l1.col(0);
  Pos2d l1_2 = l1.col(1);

  Pos2d l2_1 = l2.col(0);
  Pos2d l2_2 = l2.col(1);

  // Check if line length is zero
  if ( (l1_1-l1_2).isZero() || (l2_1-l2_2).isZero() ) {
//    std::cout << "line length is zero" << std::endl;
    return false;
  }

  Pos2d v1 = l1_2-l1_1;
//  std::cout << "v1 norm: " << v1.norm();
  v1 = v1/v1.norm();

  Pos2d v2 = l2_2-l2_1;
//  std::cout << "v2 norm: " << v2.norm();
  v2 = v2/v2.norm();

  // Check if v1 and v2 are parallel (matrix would not be invertible)
  Eigen::Matrix2d A;
  Pos2d x;
  x(0) = -1; x(1) = -1;
  A << -v1, v2;
  Eigen::FullPivLU<Eigen::Matrix2d> Apiv(A);
  if (Apiv.rank() == 2) {
//    std::cout << "solving A-b" << std::endl;
    x = A.lu().solve(l1_1-l2_1);
//    std::cout << "solution: " << x << std::endl;
  }

  if (x(0)>=0.0 && x(0)<=1.0) {
    intersection << l1_1(0) + x(0)*v1(0),
                    l1_1(1) + x(0)*v1(1);

//    std::cout << std::endl
//              << "intersection: " << intersection << std::endl
//              << "A: " << A << std::endl
//              << "x: " << x << std::endl
//              << "l1_1: " << l1_1 << std::endl;

    return true;
  } else {
//    std::cout << "v1 and v2 are parallel" << std::endl;
    return false;
  }
}


Eigen::Matrix<double,2,3> CoMOverSupportPolygonControlStaticGait::getSafeTriangle(const Eigen::Matrix<double,2,3>& supportTriangle) {
  Eigen::Matrix<double,2,3> safeTriangle;
  safeTriangle.setZero();

  Pos2d v1, v2;

  for (int k=0; k<3; k++) {
    Pos2d v1, v2;

    int vertex1, vertex2;
    if (k==0) {
      vertex1 = 1;
      vertex2 = 2;
    } else if (k==1) {
      vertex1 = 2;
      vertex2 = 0;
    } else if (k==2) {
      vertex1 = 0;
      vertex2 = 1;
    }

    v1 = supportTriangle.col(vertex1) - supportTriangle.col(k); v1 = v1/v1.norm();
    v2 = supportTriangle.col(vertex2) - supportTriangle.col(k); v2 = v2/v2.norm();

    double phiv1v2 = acos(v1.dot(v2));
    safeTriangle.col(k) = supportTriangle.col(k)+delta_/sin(phiv1v2)*(v1+v2);
  }

  return safeTriangle;
}


void CoMOverSupportPolygonControlStaticGait::updateSwingLegsIndexes() {
  swingLegIndexNow_ = getIndexOfSwingLeg();

  if (swingLegIndexNow_ != -1) {
//    swingLegIndexLast_ = swingLegIndexNow_;
//    swingLegIndexNext_ = getNextSwingFoot(swingLegIndexNow_);

    swingLegIndexLast_ = getNextSwingFoot(swingLegIndexNow_);
    swingLegIndexNext_ = getNextSwingFoot(swingLegIndexLast_);

  }
  else {
    swingFootChanged_ = true;
  }

  swingLegIndexOverNext_ = getNextSwingFoot(swingLegIndexNext_);

  std::cout << "*******************" << std::endl;
  std::cout << "last:  " << swingLegIndexLast_ << std::endl;
  std::cout << "now:   " << swingLegIndexNow_ << std::endl;
  std::cout << "next:  " << swingLegIndexNext_ << std::endl;
  std::cout << "next2: " << swingLegIndexOverNext_ << std::endl;
}



int CoMOverSupportPolygonControlStaticGait::getIndexOfSwingLeg() {
  int swingLeg = -1;
  for (auto leg: *legs_) {
    if (leg->getSwingPhase() != -1) {
      swingLeg = leg->getId();
    }
  }
  return swingLeg;
}


int CoMOverSupportPolygonControlStaticGait::getNextSwingFoot(const int currentSwingFoot) {
  int nextSwingFoot = -1;

  int currentSwingFootVectorIndex = std::find(swingOrder_.begin(), swingOrder_.end(), currentSwingFoot) - swingOrder_.begin();

  if (currentSwingFootVectorIndex == 3) nextSwingFoot = swingOrder_[0];
  else nextSwingFoot = swingOrder_[currentSwingFootVectorIndex+1];

  return nextSwingFoot;
}


std::vector<int> CoMOverSupportPolygonControlStaticGait::getDiagonalElements(int swingLeg) {
  std::vector<int> diagonalSwingLegs(2);

//  switch(swingLeg) {
//    case(0):
//    case(3):
//      diagonalSwingLegs[0] = 0;
//      diagonalSwingLegs[1] = 3;
//      break;
//    case(1):
//    case(2):
//      diagonalSwingLegs[0] = 1;
//      diagonalSwingLegs[1] = 2;
//      break;
//    default:
//      break;
  switch(swingLeg) {
    case(0):
      diagonalSwingLegs[0] = 0;
      diagonalSwingLegs[1] = 1;
      break;
    case(1):
      diagonalSwingLegs[0] = 0;
      diagonalSwingLegs[1] = 2;
      break;
    case(2):
      diagonalSwingLegs[0] = 0;
      diagonalSwingLegs[1] = 2;
      break;
    case(3):
      diagonalSwingLegs[0] = 1;
      diagonalSwingLegs[1] = 2;
      break;
    default:
      break;
  }

//  std::cout << "***********" << std::endl;
//  std::cout << "swing leg: " << swingLeg << std::endl;
//  std::cout << "diagonal elements: " <<diagonalSwingLegs[0] << " " << diagonalSwingLegs[1] << std::endl;

  return diagonalSwingLegs;
}


} /* namespace loco */
