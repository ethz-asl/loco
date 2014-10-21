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
    resumeWalking_(false)
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
  }

  serviceTestCounter_ = 0;


#ifdef USE_ROS_SERVICE
   printf("FootPlacementStrategyStaticGait: uses ros service\n");
#endif
}


FootPlacementStrategyStaticGait::~FootPlacementStrategyStaticGait() {

}


void FootPlacementStrategyStaticGait::setCoMControl(CoMOverSupportPolygonControlBase* comControl) {
  comControl_ = static_cast<CoMOverSupportPolygonControlStaticGait*>(comControl);
}


bool FootPlacementStrategyStaticGait::initialize(double dt) {
  FootPlacementStrategyFreePlane::initialize(dt);

  goToStand_ = true;
  resumeWalking_ = false;

  nextSwingLegId_ = comControl_->getNextSwingLeg();

  for (auto leg: *legs_) {
    positionWorldToValidatedDesiredFootHoldInWorldFrame_[leg->getId()] = leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame();
    positionCenterOfValidatedFeetToDefaultFootInControlFrame_[leg->getId()] = leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame();
    terrain_->getHeight(positionCenterOfValidatedFeetToDefaultFootInControlFrame_[leg->getId()]);
    positionWorldToCenterOfValidatedFeetInWorldFrame_ += positionWorldToValidatedDesiredFootHoldInWorldFrame_[leg->getId()]/legs_->size();
    positionWorldToFootHoldInWorldFrame_[leg->getId()] = positionCenterOfValidatedFeetToDefaultFootInControlFrame_[leg->getId()];
    newFootHolds_[leg->getId()] = positionWorldToValidatedDesiredFootHoldInWorldFrame_[leg->getId()];
  }

  return true;
}


void FootPlacementStrategyStaticGait::sendValidationRequest(const int legId, const Position& positionWorldToDesiredFootHoldInWorldFrame) {
#ifdef USE_ROS_SERVICE
  robotUtils::RosService::ServiceType srv;
    if (footholdRosService_.isReadyForRequest())  {
      std::cout << "validating position: " << positionWorldToDesiredFootHoldInWorldFrame << " for leg: " << legId << std::endl;
        foothold_finding_msg::Foothold foothold;
        foothold.header.seq = serviceTestCounter_;
        foothold.header.frame_id = "/starleth/odometry";
        struct timeval timeofday;
        gettimeofday(&timeofday,NULL);
        foothold.header.stamp.sec  = timeofday.tv_sec;
        foothold.header.stamp.nsec = timeofday.tv_usec * 1000;
        foothold.stepNumber = 2;   // step number

        std::string legName;
        switch(legId) {
          case(0): legName = "LF"; break;
          case(1): legName = "RF"; break;
          case(2): legName = "LH"; break;
          case(3): legName = "RH"; break;
          default: break;
        }

//        std::cout << "legname: " <<legName << std::endl;

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
        }
    }// if serviceTestCounter
    else {
      std::cout << "Service is not ready for request!" << std::endl;
    }
#endif
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
//         std::cout << "header.seq: " << srv.response.adaptedFootholds[0].header.seq << std::endl;
//         std::cout << "header.stamp: " << srv.response.adaptedFootholds[0].header.stamp.sec << "." << srv.response.adaptedFootholds[0].header.stamp.nsec << std::endl;
//         std::cout << "data: " << srv.response.adaptedFootholds[0].type.data << std::endl;

         // save validated foothold
         positionWorldToValidatedFootHoldInWorldFrame.x() = srv.response.adaptedFootholds[0].pose.position.x;
         positionWorldToValidatedFootHoldInWorldFrame.y() = srv.response.adaptedFootholds[0].pose.position.y;
         positionWorldToValidatedFootHoldInWorldFrame.z() = srv.response.adaptedFootholds[0].pose.position.z;

         int legId = -1;

         std::string dataField = srv.response.adaptedFootholds[0].type.data;

         switch(srv.response.adaptedFootholds[0].flag) {
           case(0):
               std::cout << "unknown" << std::endl;
		if ( dataField.compare("LF") == 0 ) {
                legId = 0;
              } else if ( dataField.compare("RF") == 0 ) {
                legId = 1;
              } else if ( dataField.compare("LH") == 0 ) {
                legId = 2;
              } else if ( dataField.compare("RH") == 0 ) {
                legId = 3;
              }
           break;
           case(1):
                std::cout << "do not change" << std::endl;
           break;
           case(2):{
		std::cout << "verified" << std::endl;
//             std::cout << "data: " << srv.response.adaptedFootholds[0].type.data << std::endl;
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
//           std::cout << "rec state: " << (int)srv.response.adaptedFootholds[0].flag << std::endl;
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

  if (resumeWalking_) {
    if (legs_->getLegById(comControl_->getLastSwingLeg())->getSwingPhase() == -1 ) {
      legs_->getLegById(comControl_->getLastSwingLeg())->setIsInStandConfiguration(false);
    }
  }


  /************************************************
   * Generate a foothold if all feet are grounded *
   ************************************************/
  if (comControl_->getSwingFootChanged() && !footHoldPlanned_) {

    if (resumeWalking_) {


      if (DEBUG_FPS) std::cout << "plan for leg id: " << nextSwingLegId_ << std::endl;

      // generate foothold
      if (DEBUG_FPS) std::cout << "generating foot hold..." << std::endl;
      generateFootHold(nextSwingLeg);
      if (DEBUG_FPS) std::cout << "...done!" << std::endl;
      footHoldPlanned_ = true;

      // send the generated foothold to the ROS validation service
      sendValidationRequest(nextSwingLegId_, positionWorldToFootHoldInWorldFrame_[nextSwingLegId_]);

      // old code
#ifndef USE_ROS_SERVICE
      positionWorldToValidatedDesiredFootHoldInWorldFrame_[nextSwingLegId_] = positionWorldToFootHoldInWorldFrame_[nextSwingLegId_];
      comControl_->setFootHold(nextSwingLegId_, positionWorldToValidatedDesiredFootHoldInWorldFrame_[nextSwingLegId_]);
#endif

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
      legs_->getLegById(comControl_->getLastSwingLeg())->setIsInStandConfiguration(true);
      legs_->getLegById(comControl_->getLastSwingLeg())->setIsSupportLeg(true);
    } // if go to stand


  }
  if (comControl_->getAllFeetGrounded()) {
    footHoldPlanned_ = false;
  }
  /************************************************/


  /*************************************************
   * Check if the validation service has an answer *
   *************************************************/
  if (resumeWalking_) {
    // validate foothold
    if (DEBUG_FPS) std::cout << "validating foot hold..." << std::endl;
    getValidatedFootHold(nextSwingLegId_, positionWorldToFootHoldInWorldFrame_[nextSwingLegId_]);
    if (DEBUG_FPS) std::cout << "...done!" << std::endl;
  }
  /*************************************************/


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
Position FootPlacementStrategyStaticGait::getValidatedFootHold(const int legId, const Position& positionWorldToDesiredFootHoldInWorldFrame) {
  Position positionWorldToValidatedFootHoldInWorldFrame = positionWorldToDesiredFootHoldInWorldFrame;

  if (!getValidationResponse(positionWorldToValidatedFootHoldInWorldFrame)) {
//    std::cout << "" << std::endl;
  }

  return positionWorldToValidatedFootHoldInWorldFrame;
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
    //positionWorldToCenterOfFeetAtLiftOffInWorldFrame += legAuto->getStateLiftOff()->getPositionWorldToFootInWorldFrame()/legs_->size();
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



} /* namespace loco */

