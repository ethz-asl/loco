/********************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014,  C. Dario Bellicoso, Christian Gehring, Péter Fankhauser, Stelian Coros
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*!
* @file     CoMOverSupportPolygonControlStaticGait.hpp
* @author   C. Dario Bellicoso
* @date     Oct 7, 2014
* @brief
*/

#ifndef LOCO_COMOVERSUPPORTPOLYGONCONTROLSTATICGAIT_HPP_
#define LOCO_COMOVERSUPPORTPOLYGONCONTROLSTATICGAIT_HPP_

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlBase.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"
#include "loco/common/TorsoBase.hpp"
#include "robotUtils/filters/FirstOrderFilter.hpp"

namespace loco {

class CoMOverSupportPolygonControlStaticGait: public CoMOverSupportPolygonControlBase {
 public:

  enum DefaultSafeTriangleDelta {DeltaForward, DeltaBackward};

  typedef Eigen::Matrix<double,2,4> FeetConfiguration;
  typedef Eigen::Matrix<double,2,3> SupportTriangle;
  typedef Eigen::Matrix<double,2,2> Line;
  typedef Eigen::Vector2d Pos2d;

  CoMOverSupportPolygonControlStaticGait(LegGroup* legs, TorsoBase* torso);
  virtual ~CoMOverSupportPolygonControlStaticGait();

  virtual void advance(double dt);
  virtual bool initialize();

  virtual const Position& getPositionWorldToDesiredCoMInWorldFrame() const;

  Eigen::Matrix<double,2,3> getSupportTriangleCurrent() const;
  Eigen::Matrix<double,2,3> getSupportTriangleNext() const;
  Eigen::Matrix<double,2,3> getSupportTriangleOverNext() const;

  Eigen::Matrix<double,2,3> getSafeTriangleCurrent() const;
  Eigen::Matrix<double,2,3> getSafeTriangleNext() const;
  Eigen::Matrix<double,2,3> getSafeTriangleOverNext() const;

  virtual bool setToInterpolated(const CoMOverSupportPolygonControlBase& supportPolygon1, const CoMOverSupportPolygonControlBase& supportPolygon2, double t);

  virtual int getNextSwingLeg();
  virtual int getBeforeLandingSwingLeg();
  virtual int getOverNextSwingLeg();
  virtual int getCurrentSwingLeg();

  virtual void setFootHold(int legId, Position footHold);


  virtual bool getSwingFootChanged();
  virtual bool getAllFeetGrounded();

  virtual void setDelta(double delta);
  virtual void setDelta(DefaultSafeTriangleDelta defaultSafeTriangle);

  /*! Loads the parameters from the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  virtual bool loadParameters(TiXmlHandle &hParameterSet);
  virtual bool loadParametersStaticGait(TiXmlHandle &hParameterSet);

  virtual void setIsInStandConfiguration(bool isInStandConfiguration);
  virtual bool isInStandConfiguration() const;

  virtual bool isSafeToResumeWalking();

  virtual void printState();

 protected:

  TorsoBase* torso_;
  FootPlacementStrategyBase* footPlacementStrategy_;

  std::vector<Position> plannedFootHolds_;

  int swingLegIndexNow_;
  int swingLegIndexNext_;
  int swingLegIndexBeforeLanding_;
  int swingLegIndexOverNext_;

  double defaultDeltaForward_, defaultDeltaBackward_,
         defaultFilterTimeConstant_;

  bool isInStandConfiguration_;
  bool isSafeToResumeWalking_;

  bool makeShift_;
  Pos2d comTarget_;

  Eigen::Matrix<double,2,3> supportTriangleCurrent_, supportTriangleNext_, supportTriangleOverNext_;
  Eigen::Matrix<double,2,3> safeTriangleCurrent_, safeTriangleNext_, safeTriangleOverNext_;

  FeetConfiguration feetConfigurationCurrent_, feetConfigurationNext_;

  bool swingFootChanged_;
  bool allFeetGrounded_;

  //! The swing sequence based on the gait plan
  std::vector<int> swingOrder_;

  //! Distance between the safe triangle and the support triangle segments
  double delta_;

  //! The desired CoM position in world frame
//  Position positionWorldToDesiredCoMInWorldFrame_;

  //! Get the next stance feet positions based on the gait planner
  Eigen::Matrix<double,2,4> getNextStanceConfig(const FeetConfiguration& currentStanceConfig, int steppingFoot);

  //! Get safe triangle from support triangle
  Eigen::Matrix<double,2,3> getSafeTriangle(const Eigen::Matrix<double,2,3>& supportTriangle);

  //! Get the next swing foot based on the gait sequence
  int getNextSwingFoot(const int currentSwingFoot);

  //! Find the intersection (if it exists) between two lines
  bool lineIntersect(const Line& l1, const Line& l2, Pos2d& intersection);

  //! Get the index of the current swing leg
  int getIndexOfSwingLeg();

  std::vector<int> getDiagonalElements(int swingLeg);

  void updateSwingLegsIndexes();

  void updateSafeSupportTriangles();

  robotUtils::FirstOrderFilter *filterCoMX_, *filterCoMY_;
  double filterInputCoMX_, filterInputCoMY_;
  double filterOutputCoMX_, filterOutputCoMY_;

};

} /* namespace loco */

#endif /* LOCO_COMOVERSUPPORTPOLYGONCONTROLSTATICGAIT_HPP_ */
