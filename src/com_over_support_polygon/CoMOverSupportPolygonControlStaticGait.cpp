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
    positionWorldToDesiredCoMInWorldFrame_(),
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

  // Reset Eigen variables
  homePos_.setZero();
  comStep_.setZero();
  feetConfigurationCurrent_.setZero();
  feetConfigurationNext_.setZero();
  feetConfigurationOnverNext_.setZero();

  delta_ = 0.1;
}


CoMOverSupportPolygonControlStaticGait::~CoMOverSupportPolygonControlStaticGait() {

}


bool CoMOverSupportPolygonControlStaticGait::initialize() {
  positionWorldToDesiredCoMInWorldFrame_ = Position(0.0,0.0,0.0);

  feetConfigurationCurrent_.setZero();
  feetConfigurationNext_.setZero();
  feetConfigurationOnverNext_.setZero();

  // save home feet positions
  for (auto leg: *legs_) {
    Position positionWorldToFootInFrame = leg->getWorldToFootPositionInWorldFrame();
    homePos_.col(leg->getId()) << positionWorldToFootInFrame.x(), positionWorldToFootInFrame.y();
  }

  return true;
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


    updateSwingLegsIndexes();

    if (swingFootChanged_) {
      // reset flag
      swingFootChanged_ = false;

      // update current configuration
      for (auto leg: *legs_) {
        Position positionWorldToFootInFrame = leg->getWorldToFootPositionInWorldFrame();
        feetConfigurationCurrent_.col(leg->getId()) << positionWorldToFootInFrame.x(), positionWorldToFootInFrame.y();
      }

      feetConfigurationNext_ = getNextStanceConfig(feetConfigurationCurrent_, swingLegIndexNext_);
      feetConfigurationOnverNext_ = getNextStanceConfig(feetConfigurationNext_, swingLegIndexOverNext_);




//      std::cout << "support triangle will be edged at feet indexes:";
      for (int k=0; k<legs_->size(); k++) {
        if (k != swingLegIndexNext_) {
//          std::cout << " " << k;
        }
      }
//      std::cout << std::endl;


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





Eigen::Matrix<double,2,4> CoMOverSupportPolygonControlStaticGait::getNextStanceConfig(const FeetConfiguration& currentStanceConfig, int steppingFoot) {
  FeetConfiguration nextStanceConfig;
  nextStanceConfig.setZero();

  // Find center of stance feet
  Pos2d currentFeetCenter;
  for (auto leg: *legs_) {
    Pos2d footPosition;
    footPosition << leg->getWorldToFootPositionInWorldFrame().x(), leg->getWorldToFootPositionInWorldFrame().y();
    currentFeetCenter += footPosition/legs_->size();
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
    return false;
  }

  Pos2d v1 = l1_2-l1_1;
  v1 = v1/v1.norm();

  Pos2d v2 = l2_2-l2_1;
  v2 = v2/v2.norm();

  // Check if v1 and v2 are parallel (matrix would not be invertible)
  if ( ! (abs(v1.dot(v2)-1.0) < 1e-6) ) {
    Eigen::Matrix2d A;
    A << -v1, v2;
    Pos2d x = A.lu().solve(l1_1-l2_1);
    intersection << l1_1(0) + x(0)*v1(0),
                    l1_1(1) + x(1)*v1(1);
    return true;
  }
  return false;
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

  switch(swingLeg) {
    case(0):
    case(3):
      diagonalSwingLegs[0] = 0;
      diagonalSwingLegs[1] = 3;
      break;
    case(1):
    case(2):
      diagonalSwingLegs[0] = 1;
      diagonalSwingLegs[1] = 2;
      break;
    default:
      break;
  }
  return diagonalSwingLegs;
}


} /* namespace loco */
