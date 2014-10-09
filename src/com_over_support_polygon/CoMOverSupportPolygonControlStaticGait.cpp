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
    swingFootChanged_(false)
{
  // Leg swing order
  swingOrder_[0] = 0;
  swingOrder_[1] = 3;
  swingOrder_[2] = 1;
  swingOrder_[3] = 2;

  swingLegIndexLast_ = swingOrder_[2];
  swingLegIndexNext_ = getNextSwingFoot(swingLegIndexLast_);
  swingLegIndexOverNext_= getNextSwingFoot(swingLegIndexNext_);

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

  delta_ = 0.05;
}


CoMOverSupportPolygonControlStaticGait::~CoMOverSupportPolygonControlStaticGait() {

}


bool CoMOverSupportPolygonControlStaticGait::initialize() {
  positionWorldToDesiredCoMInWorldFrame_ = torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
  positionWorldToDesiredCoMInWorldFrame_.z() = 0.0;

  feetConfigurationCurrent_.setZero();
  feetConfigurationNext_.setZero();
  feetConfigurationOverNext_.setZero();

  // save home feet positions
  for (auto leg: *legs_) {
    Position positionWorldToFootInFrame = leg->getWorldToFootPositionInWorldFrame();
    homePos_.col(leg->getId()) << positionWorldToFootInFrame.x(), positionWorldToFootInFrame.y();
  }

  return true;
}


void CoMOverSupportPolygonControlStaticGait::advance(double dt) {
    updateSwingLegsIndexes();

    RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
    LinearVelocity desiredLinearVelocityInWorldFrame = orientationWorldToControl.inverseRotate(torso_->getDesiredState().getLinearVelocityBaseInControlFrame());
    Eigen::Vector2d desiredLinearVelocity;
    desiredLinearVelocity << desiredLinearVelocityInWorldFrame.x(),
                             desiredLinearVelocityInWorldFrame.y();

    comStep_ = 0.5*desiredLinearVelocity*legs_->getLeftForeLeg()->getStanceDuration();

    std::cout << "com step: " << comStep_ << std::endl;

    if (1+0*swingFootChanged_) {
      // reset flag
      swingFootChanged_ = false;

      // update current configuration
      for (auto leg: *legs_) {
        Position positionWorldToFootInFrame = leg->getWorldToFootPositionInWorldFrame();
        feetConfigurationCurrent_.col(leg->getId()) << positionWorldToFootInFrame.x(), positionWorldToFootInFrame.y();
      }

      feetConfigurationNext_ = getNextStanceConfig(feetConfigurationCurrent_, swingLegIndexNext_);
      feetConfigurationOverNext_ = getNextStanceConfig(feetConfigurationNext_, swingLegIndexOverNext_);

//      std::cout << "*********************************************" << std::endl;
//      std::cout << "current feet:  " << std::endl << feetConfigurationCurrent_ << std::endl;
//      std::cout << "next feet:     " << std::endl << feetConfigurationNext_ << std::endl;
//      std::cout << "overnext feet: " << std::endl << feetConfigurationOverNext_ << std::endl;

      // Get support triangles
//      std::cout << "*********************************************" << std::endl;
//      std::cout << "current swing leg: " << swingLegIndexLast_ << std::endl;
//      std::cout << "current support legs: ";
      int j=0;
      for (int k=0; k<legs_->size(); k++) {
        if (k != swingLegIndexLast_) {
//          std::cout << " " << k;
          supportTriangleCurrent_.col(j) = feetConfigurationCurrent_.col(k);
          j++;
        }
      }
//      std::cout << std::endl;

      safeTriangleCurrent_ = getSafeTriangle(supportTriangleCurrent_);
//      std::cout << "current support: " << std::endl << supportTriangleCurrent_ << std::endl;
//      std::cout << "current safe:    " << std::endl << safeTriangleCurrent_ << std::endl;


//      std::cout << "*********************************************" << std::endl;
//      std::cout << "next swing leg: " << swingLegIndexNext_ << std::endl;
//      std::cout << "next support legs: ";
      j=0;
      for (int k=0; k<legs_->size(); k++) {
        if (k != swingLegIndexNext_) {
//          std::cout << " " << k;
          supportTriangleNext_.col(j) = feetConfigurationNext_.col(k);
          j++;
        }
      }
//      std::cout << std::endl;
      safeTriangleNext_ = getSafeTriangle(supportTriangleNext_);


//      std::cout << "*********************************************" << std::endl;
//      std::cout << "over next swing leg: " << swingLegIndexOverNext_ << std::endl;
//      std::cout << "over next support legs: ";
      j=0;
      for (int k=0; k<legs_->size(); k++) {
        if (k != swingLegIndexOverNext_) {
//          std::cout << " " << k;
          supportTriangleOverNext_.col(j) = feetConfigurationOverNext_.col(k);
          j++;
        }
      }
//      std::cout << std::endl;
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
        std::cout << "intersection between current and next" << std::endl;
        positionWorldToDesiredCoMInWorldFrame_ << intersection[0],
                                                  intersection[1],
                                                  0.0;
      } else if (lineIntersect(lineSafeNext, lineSafeOverNext, intersection)) {
        std::cout << "intersection between next and overnext" << std::endl;
        positionWorldToDesiredCoMInWorldFrame_ << intersection[0],
                                                  intersection[1],
                                                  0.0;
      } else {

        std::cout << "no intersection!" << std::endl;
      }

      std::cout << "des pos: " << positionWorldToDesiredCoMInWorldFrame_ << std::endl;

    }


    /*
     * TEST LINE INTERSECTION
     */
//  Eigen::Vector2d p1_1;
//  p1_1 << -1.0, 0.0;
//  Eigen::Vector2d p1_2;
//  p1_2 << 1.0, 0.0;
//
//  Eigen::Vector2d p2_1;
//  p2_1 << 0.5, -1.0;
//  Eigen::Vector2d p2_2;
//  p2_2 << 0.5, 1.0;
//
//  Eigen::Matrix<double,2,2> l1;
//  l1 << p1_1, p1_2;
//
//  Eigen::Matrix<double,2,2> l2;
//  l2 << p2_1, p2_2;
//
//  Eigen::Vector2d inters;
//  inters << 2.0, 2.0;
//
//  if ( lineIntersect(l1, l2, inters) ) {
//    std::cout << "found intersection: " << inters << std::endl;
//  }
//  else {
//    std::cout << "no intersection." << std::endl;
//  }

    /*
     * TEST SAFE TRIANGLE
     */
//    Eigen::Matrix<double,2,3> triangle;
//    triangle << 0.0, 1.0, 0.0,
//                0.0, 0.0, 1.0;
//
//    Eigen::Matrix<double,2,3> safetriangle = getSafeTriangle(triangle);
//    std::cout << "safe: " << safetriangle << std::endl;

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
  FeetConfiguration nextStanceConfig;
  nextStanceConfig.setZero();

  // Find center of stance feet
  Pos2d currentFeetCenter;
  for (auto leg: *legs_) {
    Pos2d footPosition;
//    footPosition << leg->getWorldToFootPositionInWorldFrame().x(), leg->getWorldToFootPositionInWorldFrame().y();
//    currentFeetCenter += footPosition/legs_->size();
    currentFeetCenter += currentStanceConfig.col(leg->getId())/legs_->size();
  }

  // Find new feet center (after step will be made)
  Pos2d newFeetCenter;
  newFeetCenter = currentFeetCenter + comStep_;

  // Find default feet positions in new stance config
  nextStanceConfig = currentStanceConfig;
  nextStanceConfig.col(steppingFoot) = newFeetCenter + homePos_.col(steppingFoot);

  // Update swing foot position in new stance config and correct it if needed
  Pos2d newFootPos;
  Pos2d curFootPos;
  curFootPos = currentStanceConfig.col(steppingFoot);
  newFootPos = nextStanceConfig.col(steppingFoot);
  newFootPos = curFootPos + (newFootPos-curFootPos)/(newFootPos-curFootPos).norm()*std::min((newFootPos-curFootPos).norm(), maxComStep_);

  nextStanceConfig.col(steppingFoot) = newFootPos;

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


const Position& CoMOverSupportPolygonControlStaticGait::getPositionWorldToDesiredCoMInWorldFrame() const {
  return positionWorldToDesiredCoMInWorldFrame_;
}


void CoMOverSupportPolygonControlStaticGait::updateSwingLegsIndexes() {
  swingLegIndexNow_ = getIndexOfSwingLeg();

  if (swingLegIndexNow_ != -1) {
    swingLegIndexLast_ = swingLegIndexNow_;
    swingLegIndexNext_ = getNextSwingFoot(swingLegIndexNow_);
  }
  else {
    swingFootChanged_ = true;
  }

  swingLegIndexOverNext_ = getNextSwingFoot(swingLegIndexNext_);

//  std::cout << "*******************" << std::endl;
//  std::cout << "last:  " << swingLegIndexLast_ << std::endl;
//  std::cout << "now:   " << swingLegIndexNow_ << std::endl;
//  std::cout << "next:  " << swingLegIndexNext_ << std::endl;
//  std::cout << "next2: " << swingLegIndexOverNext_ << std::endl;
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
