/*
 * FootPlacementStrategyStaticGait.cpp
 *
 *  Created on: Oct 6, 2014
 *      Author: Dario Bellicoso
 */


#include "loco/foot_placement_strategy/FootPlacementStrategyStaticGait.hpp"


/****************************
 * Includes for ROS service *
 ****************************/
#include <ctime>
#include <ratio>
#include <chrono>
/****************************/

const bool DEBUG_FPS = false;

namespace loco {

FootPlacementStrategyStaticGait::FootPlacementStrategyStaticGait(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain) :
    FootPlacementStrategyFreePlane(legs, torso, terrain),
    positionCenterOfValidatedFeetToDefaultFootInControlFrame_(legs_->size()),
    positionWorldToValidatedDesiredFootHoldInWorldFrame_(legs_->size()),
    comControl_(nullptr),
    newFootHolds_(legs_->size()),
    footHoldPlanned_(false),
    nextSwingLegId_(3),
    goToStand_(true),
    resumeWalking_(false),
    mustValidateNextFootHold_(false),
    validationRequestSent_(false),
    validationReceived_(false),
    useRosService_(false),
    defaultMaxStepLength_(0.0),
    footStepNumber_(0),
    rosWatchdogCounter_(0.0),
    rosWatchdogLimit_(0.125*(legs_->getLeftForeLeg()->getStanceDuration()+legs_->getLeftForeLeg()->getStanceDuration())),
    firstFootHoldAfterStand_(legs_->size())
{

  stepInterpolationFunction_.clear();
  stepInterpolationFunction_.addKnot(0, 0);
  stepInterpolationFunction_.addKnot(1.0, 1);

  stepFeedbackScale_ = 0.0;

  positionWorldToCenterOfValidatedFeetInWorldFrame_ = Position();

//  positionBaseOnTerrainToDefaultFootInControlFrame_[0] = Position(0.25, 0.18, 0.0);
//  positionBaseOnTerrainToDefaultFootInControlFrame_[1] = Position(0.25, -0.18, 0.0);
//  positionBaseOnTerrainToDefaultFootInControlFrame_[2] = Position(-0.25, 0.18, 0.0);
//  positionBaseOnTerrainToDefaultFootInControlFrame_[3] = Position(-0.25, -0.18, 0.0);

  for (auto leg: *legs_) {
    positionCenterOfValidatedFeetToDefaultFootInControlFrame_[leg->getId()].setZero();
    positionWorldToValidatedDesiredFootHoldInWorldFrame_[leg->getId()].setZero();
    positionWorldToFootHoldInWorldFrame_[leg->getId()].setZero();
    newFootHolds_[leg->getId()].setZero();
    firstFootHoldAfterStand_[leg->getId()] = leg->isInStandConfiguration();
  }

  serviceTestCounter_ = 0;


#ifdef USE_ROS_SERVICE
  if (useRosService_) {
    printf("FootPlacementStrategyStaticGait: uses ros service\n");
  }
#endif
}


FootPlacementStrategyStaticGait::~FootPlacementStrategyStaticGait() {

}


void FootPlacementStrategyStaticGait::setCoMControl(CoMOverSupportPolygonControlBase* comControl) {
  comControl_ = static_cast<CoMOverSupportPolygonControlStaticGait*>(comControl);
}

bool FootPlacementStrategyStaticGait::isUsingRosService() {
  return useRosService_;
}

void FootPlacementStrategyStaticGait::setUseRosService(bool useRosService) {
  useRosService_ = useRosService;
}


bool FootPlacementStrategyStaticGait::initialize(double dt) {
  FootPlacementStrategyFreePlane::initialize(dt);

//  defaultMaxStepLength_ = 0.05;

  useRosService_ = false;

  rosWatchdogCounter_ = 0.0;
  rosWatchdogLimit_ = 0.125*7.0*0.5;
  std::cout << "ros wd limit: " << rosWatchdogLimit_ << std::endl;

  footStepNumber_ = 0;

  goToStand_ = true;
  resumeWalking_ = false;
  mustValidateNextFootHold_ = false;
  validationRequestSent_ = false;
  validationReceived_ = false;

  nextSwingLegId_ = comControl_->getNextSwingLeg();

  for (auto leg: *legs_) {
//    positionWorldToValidatedDesiredFootHoldInWorldFrame_[leg->getId()] = leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame();
//    positionCenterOfValidatedFeetToDefaultFootInControlFrame_[leg->getId()] = leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame();

    positionWorldToValidatedDesiredFootHoldInWorldFrame_[leg->getId()] = leg->getPositionWorldToFootInWorldFrame();
    positionCenterOfValidatedFeetToDefaultFootInControlFrame_[leg->getId()] = leg->getPositionWorldToFootInWorldFrame();

    terrain_->getHeight(positionCenterOfValidatedFeetToDefaultFootInControlFrame_[leg->getId()]);
    positionWorldToCenterOfValidatedFeetInWorldFrame_ += positionWorldToValidatedDesiredFootHoldInWorldFrame_[leg->getId()]/legs_->size();
    positionWorldToFootHoldInWorldFrame_[leg->getId()] = positionCenterOfValidatedFeetToDefaultFootInControlFrame_[leg->getId()];
    newFootHolds_[leg->getId()] = positionWorldToValidatedDesiredFootHoldInWorldFrame_[leg->getId()];
  }

  return true;
}


bool FootPlacementStrategyStaticGait::sendValidationRequest(const int legId, const Position& positionWorldToDesiredFootHoldInWorldFrame) {
  bool ready = true;

#ifdef USE_ROS_SERVICE
  robotUtils::RosService::ServiceType srv;
  ready = footholdRosService_.isReadyForRequest();
    if (ready) {
      std::cout << "validating position: " << positionWorldToDesiredFootHoldInWorldFrame << " for leg: " << legId << std::endl;
        foothold_finding_msg::Foothold foothold;
        foothold.header.seq = serviceTestCounter_;
        foothold.header.frame_id = "/starleth/odometry";
        struct timeval timeofday;
        gettimeofday(&timeofday,NULL);
        foothold.header.stamp.sec  = timeofday.tv_sec;
        foothold.header.stamp.nsec = timeofday.tv_usec * 1000;
        foothold.stepNumber = ++footStepNumber_;   // step number

        std::string legName;
        switch(legId) {
          case(0): legName = "LF"; break;
          case(1): legName = "RF"; break;
          case(2): legName = "LH"; break;
          case(3): legName = "RH"; break;
          default: break;
        }

        foothold.type.data = legName;
        foothold.pose.position.x = positionWorldToDesiredFootHoldInWorldFrame.x(); // required
        foothold.pose.position.y = positionWorldToDesiredFootHoldInWorldFrame.y(); // required
        foothold.pose.position.z = positionWorldToDesiredFootHoldInWorldFrame.z();

        foothold.pose.orientation.w = 1.0; // surface normal
        foothold.pose.orientation.x = 0.0;
        foothold.pose.orientation.y = 0.0;
        foothold.pose.orientation.z = 0.0;

        foothold.flag = 0;      // 0: unknown, 1: do not change position, 2: verified, 3: bad)
        srv.request.initialFootholds.push_back(foothold);

        if (!footholdRosService_.sendRequest(srv)) {
          std::cout << "Could not send request!\n";
          return false;
        }
    }// if serviceTestCounter
    else {
//      std::cout << "Service is not ready for request!" << std::endl;
    }
#endif

  return ready;
}


bool FootPlacementStrategyStaticGait::getValidationResponse(Position& positionWorldToValidatedFootHoldInWorldFrame) {
  bool success = false;

#ifdef USE_ROS_SERVICE
  robotUtils::RosService::ServiceType srv;
  if (footholdRosService_.hasReceivedResponse()) {
    bool isValid;
    if (footholdRosService_.receiveResponse(srv, isValid)) {
      if (isValid) {
        success = true;
//        std::cout << "Received request:\n";

       if (!srv.response.adaptedFootholds.empty() ){

         foothold_finding_msg::Foothold receviedFoothold = srv.response.adaptedFootholds[0];

//         std::cout << "header.seq: " << receviedFoothold.header.seq << std::endl;
//         std::cout << "header.stamp: " << receviedFoothold.header.stamp.sec << "." << receviedFoothold.header.stamp.nsec << std::endl;
//         std::cout << "data: " << receviedFoothold.type.data << std::endl;

         // save validated foothold
         positionWorldToValidatedFootHoldInWorldFrame.x() = receviedFoothold.pose.position.x;
         positionWorldToValidatedFootHoldInWorldFrame.y() = receviedFoothold.pose.position.y;
         positionWorldToValidatedFootHoldInWorldFrame.z() = receviedFoothold.pose.position.z;

         int legId = -1;

         std::string dataField = receviedFoothold.type.data;

         switch(receviedFoothold.flag) {
           case(0):
              std::cout << "unknown" << std::endl;
              if (dataField.compare("LF") == 0) {
                legId = 0;
              } else if (dataField.compare("RF") == 0) {
                legId = 1;
              } else if (dataField.compare("LH") == 0) {
                legId = 2;
              } else if (dataField.compare("RH") == 0) {
                legId = 3;
              }
           break;
           case(1):
                std::cout << "do not change" << std::endl;
           break;
           case(2):{
		std::cout << "verified" << std::endl;
//             std::cout << "data: " << receviedFoothold.type.data << std::endl;
              if ( dataField.compare("LF") == 0 ) {
                legId = 0;
              } else if ( dataField.compare("RF") == 0 ) {
                legId = 1;
              } else if ( dataField.compare("LH") == 0 ) {
                legId = 2;
              } else if ( dataField.compare("RH") == 0 ) {
                legId = 3;
              }
//              std::cout << "leg id: " << legId << std::endl;
           }
           break;
           case(3):
               std::cout << "bad" << std::endl;
           break;
           default: break;
         }

         if (legId != -1) {
//           std::cout << "leg id:    " << legId << std::endl;
//           std::cout << "rec state: " << (int)receviedFoothold.flag << std::endl;
           positionWorldToValidatedDesiredFootHoldInWorldFrame_[legId] = positionWorldToValidatedFootHoldInWorldFrame;
           comControl_->setFootHold(legId, positionWorldToValidatedFootHoldInWorldFrame);
         }

       }

      }
      else {
        std::cout << "Received error!\n";
      }
    }
  }
  serviceTestCounter_++;
#endif

  return success;
}


bool FootPlacementStrategyStaticGait::goToStand() {
  resumeWalking_ = false;
  goToStand_ = true;

  return true;
}


bool FootPlacementStrategyStaticGait::resumeWalking() {
  goToStand_ = false;
  resumeWalking_ = true;

  return true;
}


bool FootPlacementStrategyStaticGait::advance(double dt) {
  /*******************
   * Update leg data *
   *******************/
  for (auto leg : *legs_) {
    // save the hip position at lift off for trajectory generation
    if (leg->shouldBeGrounded() ||
        (!leg->shouldBeGrounded() && leg->isGrounded() && leg->getSwingPhase() < 0.25)
    ) {
      Position positionWorldToHipAtLiftOffInWorldFrame = leg->getPositionWorldToHipInWorldFrame();
      positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()] = getPositionProjectedOnPlaneAlongSurfaceNormal(positionWorldToHipAtLiftOffInWorldFrame);
      Position positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame = positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()];

      Position positionWorldToHipOnTerrainAlongWorldZInWorldFrame = positionWorldToHipAtLiftOffInWorldFrame;
      terrain_->getHeight(positionWorldToHipOnTerrainAlongWorldZInWorldFrame);

      /*
       * WARNING: these were also updated by the event detector
       */
      leg->getStateLiftOff()->setPositionWorldToHipOnTerrainAlongWorldZInWorldFrame(positionWorldToHipOnTerrainAlongWorldZInWorldFrame);
      leg->getStateLiftOff()->setPositionWorldToFootInWorldFrame(leg->getPositionWorldToFootInWorldFrame());
      leg->getStateLiftOff()->setPositionWorldToHipInWorldFrame(leg->getPositionWorldToHipInWorldFrame());
      leg->setSwingPhase(leg->getSwingPhase());
    }

  } // for auto leg
  /*******************/


  // get pointer to next swing leg
  nextSwingLegId_ = comControl_->getNextSwingLeg();
  LegBase* nextSwingLeg = legs_->getLegById(nextSwingLegId_);

  if (resumeWalking_ && comControl_->isSafeToResumeWalking()) {
    if (nextSwingLeg->getStancePhase() < 0.75 && nextSwingLeg->getStancePhase() != -1) {
      nextSwingLeg->setIsInStandConfiguration(false);
    }
  }

  /************************************************
   * Generate a foothold if all feet are grounded *
   ************************************************/
  if (comControl_->getSwingFootChanged() && !footHoldPlanned_) {
    /*
     * This section will be evaluated only once for each stand phase
     */

    if (resumeWalking_ && comControl_->isSafeToResumeWalking()) {

      // generate foothold
      generateFootHold(nextSwingLeg);

      // Reset ROS flags after foothold generation
      mustValidateNextFootHold_ = true;
      rosWatchdogCounter_ = 0.0;
      validationRequestSent_ = false;
      validationReceived_ = false;
      footHoldPlanned_ = true;

      // send the generated foothold to the ROS validation service
//      sendValidationRequest(nextSwingLegId_, positionWorldToFootHoldInWorldFrame_[nextSwingLegId_]);

      // old code
//#ifndef USE_ROS_SERVICE
      if (!useRosService_) {
        positionWorldToValidatedDesiredFootHoldInWorldFrame_[nextSwingLegId_] = positionWorldToFootHoldInWorldFrame_[nextSwingLegId_];
        comControl_->setFootHold(nextSwingLegId_, positionWorldToValidatedDesiredFootHoldInWorldFrame_[nextSwingLegId_]);
      }
//#endif

      /**********************************************************************************************************************************
       * temporary solution:        when going backwards, use a smaller distance for safe triangle evaluation.                          *
       * possible better solution:  transition to new gait sequence, or push the center of mass nearer to the support triangle diagonal *
       **********************************************************************************************************************************/
      if (torso_->getDesiredState().getLinearVelocityBaseInControlFrame().x() >= 0.0 ) {
        comControl_->setDelta(CoMOverSupportPolygonControlStaticGait::DefaultSafeTriangleDelta::DeltaForward);
      }
      else if (torso_->getDesiredState().getLinearVelocityBaseInControlFrame().x() < 0.0) {
        comControl_->setDelta(CoMOverSupportPolygonControlStaticGait::DefaultSafeTriangleDelta::DeltaBackward);
      }
    } // if resume walking


    if (goToStand_) {
      for (auto leg: *legs_) {
        leg->setIsInStandConfiguration(true);
      }
//      legs_->getLegById(comControl_->getBeforeLandingSwingLeg())->setIsInStandConfiguration(true);
//      legs_->getLegById(comControl_->getBeforeLandingSwingLeg())->setIsSupportLeg(true);
    } // if go to stand


  }
  if (comControl_->getAllFeetGrounded()) {
    footHoldPlanned_ = false;
  }
  /************************************************/


  /*********************
   * Query ROS service *
   *********************/
//#ifdef USE_ROS_SERVICE
  if (useRosService_ && mustValidateNextFootHold_) {

    if (!validationRequestSent_) {
      validationRequestSent_ = sendValidationRequest(nextSwingLegId_, positionWorldToFootHoldInWorldFrame_[nextSwingLegId_]);
      if (validationRequestSent_) {
        std::cout << "sent request!" << std::endl;
      }
    }

    if (validationRequestSent_ && !validationReceived_) {
      if (getValidatedFootHold(nextSwingLegId_, positionWorldToFootHoldInWorldFrame_[nextSwingLegId_])) {
        std::cout << "got validation! time: " << rosWatchdogCounter_ << std::endl;
        validationReceived_ = true;
        mustValidateNextFootHold_ = false;
      }
    }

    if (rosWatchdogCounter_ >= rosWatchdogLimit_) {
      std::cout << "*******ros watchdog limit: " << rosWatchdogCounter_ << std::endl;
      mustValidateNextFootHold_ = false;
    }
    rosWatchdogCounter_ += dt;

  }
//#endif
  /*********************/


  /*********************************************
   * Decide what to do with leg based on state *
   *********************************************/
  for (auto leg : *legs_) {
    if (!leg->isSupportLeg() /*&& !leg->isInStandConfiguration()*/) {
      StateSwitcher* stateSwitcher = leg->getStateSwitcher();

      switch(stateSwitcher->getState()) {
        case(StateSwitcher::States::StanceSlipping):
        case(StateSwitcher::States::StanceLostContact):
          regainContact(leg, dt); break;

        case(StateSwitcher::States::SwingNormal):
        case(StateSwitcher::States::SwingLateLiftOff):
        /*case(StateSwitcher::States::SwingBumpedIntoObstacle):*/
          setFootTrajectory(leg);  break;

        default:
          break;
      }
    }
  } // for auto leg
  /*********************************************/


  return true;
}


void FootPlacementStrategyStaticGait::setFootTrajectory(LegBase* leg) {
    const Position positionWorldToFootInWorldFrame = getDesiredWorldToFootPositionInWorldFrame(leg, 0.0);
    leg->setDesireWorldToFootPositionInWorldFrame(positionWorldToFootInWorldFrame); // for debugging
    const Position positionBaseToFootInWorldFrame = positionWorldToFootInWorldFrame - torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
    const Position positionBaseToFootInBaseFrame  = torso_->getMeasuredState().getOrientationWorldToBase().rotate(positionBaseToFootInWorldFrame);

    leg->setDesiredJointPositions(leg->getJointPositionsFromPositionBaseToFootInBaseFrame(positionBaseToFootInBaseFrame));
}


void FootPlacementStrategyStaticGait::regainContact(LegBase* leg, double dt) {
  Position positionWorldToFootInWorldFrame =  leg->getPositionWorldToFootInWorldFrame();
  double loweringSpeed = 0.05;

  loco::Vector normalInWorldFrame;
  if (terrain_->getNormal(positionWorldToFootInWorldFrame,normalInWorldFrame)) {
    positionWorldToFootInWorldFrame -= 0.01*(loco::Position)normalInWorldFrame;
    //positionWorldToFootInWorldFrame -= (loweringSpeed*dt) * (loco::Position)normalInWorldFrame;
  }
  else  {
    throw std::runtime_error("FootPlacementStrategyStaticGait::advance cannot get terrain normal.");
  }

  leg->setDesireWorldToFootPositionInWorldFrame(positionWorldToFootInWorldFrame); // for debugging
  const Position positionWorldToBaseInWorldFrame = torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame();
  const Position positionBaseToFootInWorldFrame = positionWorldToFootInWorldFrame - positionWorldToBaseInWorldFrame;
  const Position positionBaseToFootInBaseFrame = torso_->getMeasuredState().getOrientationWorldToBase().rotate(positionBaseToFootInWorldFrame);
  leg->setDesiredJointPositions(leg->getJointPositionsFromPositionBaseToFootInBaseFrame(positionBaseToFootInBaseFrame));
}


/*
 * Generate a candidate foothold for a leg. Result will be saved in class member and returned as a Position variable.
 */
Position FootPlacementStrategyStaticGait::generateFootHold(LegBase* leg) {
  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  Position positionWorldToFootOnTerrainAtLiftOffInWorldFrame = leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame();
  terrain_->getHeight(positionWorldToFootOnTerrainAtLiftOffInWorldFrame);

  // x-y offset
  Position positionFootAtLiftOffToDesiredFootHoldInWorldFrame = orientationWorldToControl.inverseRotate(getPositionFootAtLiftOffToDesiredFootHoldInControlFrame(*leg));

  Position positionWorldToFootHoldInWorldFrame = positionWorldToFootOnTerrainAtLiftOffInWorldFrame
                                                 + positionFootAtLiftOffToDesiredFootHoldInWorldFrame;

  // orientation offset - rotate foothold position vector around world z axis according to desired angular velocity
  positionWorldToFootHoldInWorldFrame = getPositionDesiredFootHoldOrientationOffsetInWorldFrame(*leg, positionWorldToFootHoldInWorldFrame);

  // update class member and get correct terrain height at foot hold
  positionWorldToFootHoldInWorldFrame_[leg->getId()] = positionWorldToFootHoldInWorldFrame;
  terrain_->getHeight(positionWorldToFootHoldInWorldFrame_[leg->getId()]);

  return positionWorldToFootHoldInWorldFrame_[leg->getId()];
}

/*
 * Check if a desired foothold is valid. Return a validated foothold.
 */
bool FootPlacementStrategyStaticGait::getValidatedFootHold(const int legId, const Position& positionWorldToDesiredFootHoldInWorldFrame) {
  Position positionWorldToValidatedFootHoldInWorldFrame = positionWorldToDesiredFootHoldInWorldFrame;
  if (!getValidationResponse(positionWorldToValidatedFootHoldInWorldFrame)) {
//    std::cout << "" << std::endl;
    return false;
  }

  return true;
}


/*
 * Foot holds are evaluated with respect to the foot positions at liftoff.
 */
Position FootPlacementStrategyStaticGait::getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep) {
  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  Position positionWorldToFootAtLiftOffInWorldFrame = leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame();

  // get the actual (validated) step that must be taken
  Position positionWorldToValidatedFootHoldInWorldFrame = positionWorldToValidatedDesiredFootHoldInWorldFrame_[leg->getId()];
  Position positionFootAtLiftOffToValidatedDesiredFootHoldInWorldFrame = positionWorldToValidatedFootHoldInWorldFrame
                                                                         - positionWorldToFootAtLiftOffInWorldFrame;

  Position positionFootAtLiftOffToValidatedDesiredFootHoldInControlFrame = orientationWorldToControl.rotate(positionFootAtLiftOffToValidatedDesiredFootHoldInWorldFrame);

  /*
   * Interpolate on the x-y plane
   */
  double interpolationParameter = getInterpolationPhase(*leg);
  Position positionFootOnTerrainAtLiftOffToDesiredFootOnTerrainInControlFrame =
      Position(
      // x
          getHeadingComponentOfFootStep(
              interpolationParameter, 0.0,
              positionFootAtLiftOffToValidatedDesiredFootHoldInControlFrame.x(),
              const_cast<LegBase*>(leg)),
          // y
          getLateralComponentOfFootStep(
              interpolationParameter, 0.0,
              positionFootAtLiftOffToValidatedDesiredFootHoldInControlFrame.y(),
              const_cast<LegBase*>(leg)),
          // z
          0.0);

  /*
   * Interpolate height trajectory
   */
  Position positionDesiredFootOnTerrainToDesiredFootInControlFrame =  getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(*leg,
                                                                                                                                 positionFootOnTerrainAtLiftOffToDesiredFootOnTerrainInControlFrame); // z


  Position positionWorldToDesiredFootInWorldFrame = positionWorldToFootAtLiftOffInWorldFrame
                                                    + orientationWorldToControl.inverseRotate(positionFootOnTerrainAtLiftOffToDesiredFootOnTerrainInControlFrame)
                                                    + orientationWorldToControl.inverseRotate(positionDesiredFootOnTerrainToDesiredFootInControlFrame);
  return positionWorldToDesiredFootInWorldFrame;
  //---
}


Position FootPlacementStrategyStaticGait::getPositionWorldToValidatedDesiredFootHoldInWorldFrame(int legId) const {
  // todo: check if legId is in admissible range
  return positionWorldToValidatedDesiredFootHoldInWorldFrame_[legId];
}


/*
 * Interpolate height: get the vector pointing from the current interpolated foot hold to the height of the desired foot based on the interpolation phase
 */
Position FootPlacementStrategyStaticGait::getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(const LegBase& leg, const Position& positionHipOnTerrainToDesiredFootOnTerrainInControlFrame)  {
  const double interpolationParameter = getInterpolationPhase(leg);
  const double desiredFootHeight = const_cast<SwingFootHeightTrajectory*>(&swingFootHeightTrajectory_)->evaluate(interpolationParameter);

  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  Position positionHipOnTerrainToDesiredFootOnTerrainInWorldFrame = orientationWorldToControl.inverseRotate(positionHipOnTerrainToDesiredFootOnTerrainInControlFrame);

  Vector normalToPlaneAtCurrentFootPositionInWorldFrame;
  terrain_->getNormal(positionHipOnTerrainToDesiredFootOnTerrainInWorldFrame,
                      normalToPlaneAtCurrentFootPositionInWorldFrame);

  Vector normalToPlaneAtCurrentFootPositionInControlFrame = orientationWorldToControl.rotate(normalToPlaneAtCurrentFootPositionInWorldFrame);

  Position positionDesiredFootOnTerrainToDesiredFootInControlFrame = desiredFootHeight*Position(normalToPlaneAtCurrentFootPositionInControlFrame);
  return positionDesiredFootOnTerrainToDesiredFootInControlFrame;
}


Position FootPlacementStrategyStaticGait::getPositionDesiredFootHoldOrientationOffsetInWorldFrame(const LegBase& leg,
                                                                                                  const Position& positionWorldToDesiredFootHoldBeforeOrientationOffsetInWorldFrame) {
  // rotational component
  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  LocalAngularVelocity desiredAngularVelocityInControlFrame = torso_->getDesiredState().getAngularVelocityBaseInControlFrame();
  LocalAngularVelocity desiredAngularVelocityInWorldFrame = orientationWorldToControl.inverseRotate(desiredAngularVelocityInControlFrame);

  Position rho = positionWorldToDesiredFootHoldBeforeOrientationOffsetInWorldFrame - positionWorldToCenterOfValidatedFeetInWorldFrame_;

  return positionWorldToDesiredFootHoldBeforeOrientationOffsetInWorldFrame
         + Position( desiredAngularVelocityInWorldFrame.toImplementation().cross( rho.toImplementation() ) );
}


// Evaluate feed forward component
Position FootPlacementStrategyStaticGait::getPositionFootAtLiftOffToDesiredFootHoldInControlFrame(const LegBase& leg)   {
  double stanceDuration = leg.getStanceDuration();

  RotationQuaternion orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
  Position positionCenterOfFootAtLiftOffToDefaultFootInWorldFrame = orientationWorldToControl.inverseRotate(positionCenterOfValidatedFeetToDefaultFootInControlFrame_[leg.getId()]);


  /* Update center of feet at lift off */
  Position positionWorldToCenterOfValidatedFeetInWorldFrame = Position();
  for (auto legAuto: *legs_) {
    //positionWorldToCenterOfFeetAtLif;tOffInWorldFrame += legAuto->getStateLiftOff()->getPositionWorldToFootInWorldFrame()/legs_->size();
    positionWorldToCenterOfValidatedFeetInWorldFrame += positionWorldToValidatedDesiredFootHoldInWorldFrame_[legAuto->getId()]/legs_->size();
  }
  terrain_->getHeight(positionWorldToCenterOfValidatedFeetInWorldFrame);
  positionWorldToCenterOfValidatedFeetInWorldFrame_ = positionWorldToCenterOfValidatedFeetInWorldFrame;


  Position positionWorldToDefafultFootInWorldFrame = positionCenterOfFootAtLiftOffToDefaultFootInWorldFrame
                                                    + positionWorldToCenterOfValidatedFeetInWorldFrame;

  Position positionFootAtLiftOffToDefaultFootInWorldFrame = positionWorldToDefafultFootInWorldFrame
                                                            - leg.getStateLiftOff().getPositionWorldToFootInWorldFrame();
  Position positionFootAtLiftOffToDefaultFootInControlFrame = orientationWorldToControl.rotate(positionFootAtLiftOffToDefaultFootInWorldFrame);

  // heading component
  Position headingAxisOfControlFrame = Position::UnitX();
  Position positionDesiredFootOnTerrainVelocityHeadingOffsetInControlFrame = Position(torso_->getDesiredState().getLinearVelocityBaseInControlFrame().toImplementation().cwiseProduct(headingAxisOfControlFrame.toImplementation()))
                                                                      *stanceDuration*0.5;

  // lateral component
  Position lateralAxisOfControlFrame = Position::UnitY();
  Position positionDesiredFootOnTerrainVelocityLateralOffsetInControlFrame = Position(torso_->getDesiredState().getLinearVelocityBaseInControlFrame().toImplementation().cwiseProduct(lateralAxisOfControlFrame.toImplementation()))
                                                                      *stanceDuration*0.5;

  // build the desired foot step displacement
  Position positionFootAtLiftOffToDesiredFootHoldInControlFrame;
  if (!leg.isInStandConfiguration()) {
    positionFootAtLiftOffToDesiredFootHoldInControlFrame = positionFootAtLiftOffToDefaultFootInControlFrame                    // default position
                                                           + positionDesiredFootOnTerrainVelocityHeadingOffsetInControlFrame   // heading
                                                           + positionDesiredFootOnTerrainVelocityLateralOffsetInControlFrame;  // lateral

    double footStepNorm = positionFootAtLiftOffToDesiredFootHoldInControlFrame.norm();
    if ( footStepNorm > defaultMaxStepLength_) {
      Position unitVectorFootAtLiftOffToDesiredFootHoldInControlFrame = positionFootAtLiftOffToDesiredFootHoldInControlFrame/footStepNorm;
      positionFootAtLiftOffToDesiredFootHoldInControlFrame = unitVectorFootAtLiftOffToDesiredFootHoldInControlFrame*defaultMaxStepLength_;
    }

  }
  else {
    positionFootAtLiftOffToDesiredFootHoldInControlFrame = positionFootAtLiftOffToDefaultFootInControlFrame;
  }


  return positionFootAtLiftOffToDesiredFootHoldInControlFrame;
}


Position FootPlacementStrategyStaticGait::getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(const LegBase& leg) {
  const RotationQuaternion& orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();

  positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg.getId()] = getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame(leg);
//  positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg.getId()] = getPositionDesiredFootHoldOnTerrainFeedBackInControlFrame(leg);

  Position positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame = positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg.getId()]
                                                                                     /*+ positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg.getId()]*/;

  //--- save for debug
  //Position positionWorldToHipOnPlaneAlongNormalInWorldFrame = getPositionProjectedOnPlaneAlongSurfaceNormal(leg.getWorldToHipPositionInWorldFrame());

  Position positionWorldToHipVerticalOnPlaneInWorldFrame = leg.getStateLiftOff().getPositionWorldToHipInWorldFrame();
//  Position positionWorldToHipVerticalOnPlaneInWorldFrame = leg.getWorldToHipPositionInWorldFrame();
  terrain_->getHeight(positionWorldToHipVerticalOnPlaneInWorldFrame);

  positionWorldToFootHoldInWorldFrame_[leg.getId()] = positionWorldToHipVerticalOnPlaneInWorldFrame
                                                      + orientationWorldToControl.inverseRotate(positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame);

//  std::cout << "foothold components for leg: " << leg.getId() << std::endl
//            << "world to hip on t: " << positionWorldToHipVerticalOnPlaneInWorldFrame << std::endl
//            << "hip on t to foot: " << orientationWorldToControl.inverseRotate(positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame) << std::endl;
  //---

  //--- starting point for trajectory interpolation
  const Position positionHipOnTerrainAlongNormalToFootAtLiftOffInWorldFrame = leg.getStateLiftOff().getPositionWorldToFootInWorldFrame()
                                                                              -leg.getStateLiftOff().getPositionWorldToHipOnTerrainAlongNormalToSurfaceAtLiftOffInWorldFrame();
  const Position positionHipOnTerrainAlongNormalToFootAtLiftOffInControlFrame = orientationWorldToControl.rotate(positionHipOnTerrainAlongNormalToFootAtLiftOffInWorldFrame);
  //---

  double interpolationParameter = getInterpolationPhase(leg);

  Position positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame = Position(
                                                                                  // x
                                                                                  getHeadingComponentOfFootStep(interpolationParameter,
                                                                                  positionHipOnTerrainAlongNormalToFootAtLiftOffInControlFrame.x(),
                                                                                  positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame.x(),
                                                                                  const_cast<LegBase*>(&leg)),
                                                                                  // y
                                                                                  getLateralComponentOfFootStep(interpolationParameter,
                                                                                  positionHipOnTerrainAlongNormalToFootAtLiftOffInControlFrame.y(),
                                                                                  positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame.y(),
                                                                                  const_cast<LegBase*>(&leg)),
                                                                                  // z
                                                                                  0.0
                                                                                  );

  return positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame;
}


bool FootPlacementStrategyStaticGait::loadParameters(const TiXmlHandle& handle) {

  bool success = FootPlacementStrategyInvertedPendulum::loadParameters(handle);

  TiXmlElement* pElem;

  /* desired */
  TiXmlHandle hFPS(handle.FirstChild("FootPlacementStrategy").FirstChild("StaticGait"));
  pElem = hFPS.Element();
  if (!pElem) {
    printf("*******Could not find FootPlacementStrategy:StaticGait\n");
    return false;
  }

  pElem = hFPS.FirstChild("FootHoldLimits").Element();
  if (pElem->QueryDoubleAttribute("defaultMaxStepLength", &defaultMaxStepLength_)!=TIXML_SUCCESS) {
    printf("*******Could not find FootHoldLimits:defaultMaxStepLength\n");
    return false;
  }

  return success;

}



} /* namespace loco */

