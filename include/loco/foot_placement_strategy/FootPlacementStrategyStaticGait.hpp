/*****************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, C. Dario Bellicoso, Christian Gehring, Péter Fankhauser, Stelian Coros
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
* @file     FootPlacementStrategyStaticGait.hpp
* @author   C. Dario Bellicoso, Christian Gehring
* @date     Oct 6, 2014
* @brief
*/
#ifndef LOCO_FOOTPLACEMENTSTRATEGYSTATICGAIT_HPP_
#define LOCO_FOOTPLACEMENTSTRATEGYSTATICGAIT_HPP_


#include "loco/foot_placement_strategy/FootPlacementStrategyFreePlane.hpp"

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlStaticGait.hpp"

#include "loco/common/TorsoBase.hpp"
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/LegGroup.hpp"


/****************************
 * Includes for ROS service *
 ****************************/
//#undef USE_ROS_SERVICE
//#define USE_ROS_SERVICE
#ifdef USE_ROS_SERVICE
#include "RosService.hpp"
#endif
/****************************/


namespace loco {

class FootPlacementStrategyStaticGait: public FootPlacementStrategyFreePlane {
 public:
  FootPlacementStrategyStaticGait(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain);
  virtual ~FootPlacementStrategyStaticGait();

  virtual bool advance(double dt);
  virtual bool initialize(double dt);

  virtual Position getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep);
  virtual Position getPositionFootAtLiftOffToDesiredFootHoldInControlFrame(const LegBase& leg);
  virtual Position getPositionDesiredFootHoldOrientationOffsetInWorldFrame(const LegBase& leg, const Position& positionWorldToDesiredFootHoldBeforeOrientationOffsetInWorldFrame);
  virtual Position getPositionWorldToValidatedDesiredFootHoldInWorldFrame(int legId) const;
  virtual Position getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(const LegBase& leg, const Position& positionHipOnTerrainToDesiredFootOnTerrainInControlFrame);

  virtual void setCoMControl(CoMOverSupportPolygonControlBase* comControl);

  virtual bool goToStand();
  virtual bool resumeWalking();

  virtual bool loadParameters(const TiXmlHandle& handle);

  virtual bool isUsingRosService();
  virtual void setUseRosService(bool useRosService);

  std::vector<Position> positionWorldToInterpolatedFootPositionInWorldFrame_;


#ifdef USE_ROS_SERVICE
  robotUtils::RosService footholdRosService_;
#endif
  int serviceTestCounter_;

 protected:

  void initLogger();

  bool goToStand_, resumeWalking_;
  bool mustValidateNextFootHold_;
  bool validationRequestSent_,validationReceived_;
  bool useRosService_;
  double rosWatchdogCounter_;
  double rosWatchdogLimit_;

  int footStepNumber_;


  std::vector<bool> firstFootHoldAfterStand_;
  bool footHoldPlanned_;

  CoMOverSupportPolygonControlStaticGait* comControl_;

  Position positionWorldToCenterOfValidatedFeetInWorldFrame_;
  std::vector<Position> positionCenterOfValidatedFeetToDefaultFootInControlFrame_;
  std::vector<Position> positionWorldToValidatedDesiredFootHoldInWorldFrame_;

  std::vector<Position> positionWorldToStartOfFootTrajectoryInWorldFrame_;

  std::vector<Position> newFootHolds_;

  virtual void setFootTrajectory(LegBase* leg);
  virtual void regainContact(LegBase* leg, double dt);

  virtual Position generateFootHold(LegBase* leg);

  virtual bool getValidatedFootHold(const int legId, const Position& positionWorldToDesiredFootHoldInWorldFrame);

  virtual bool sendValidationRequest(const int legId, const Position& positionWorldToDesiredFootHoldInWorldFrame);
  virtual bool getValidationResponse(Position& positionWorldToValidatedFootHoldInWorldFrame);
  int nextSwingLegId_;

  double defaultMaxStepLength_;

};

} /* namespace loco */


#endif /* LOCO_FOOTPLACEMENTSTRATEGYSTATICGAIT_HPP_ */
