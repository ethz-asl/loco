/*
 * FootPlacementStrategyStaticGait.hpp
 *
 *  Created on: Oct 6, 2014
 *      Author: Dario Bellicoso
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


#ifdef USE_ROS_SERVICE
  robotUtils::RosService footholdRosService_;
#endif
  int serviceTestCounter_;

 protected:


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
