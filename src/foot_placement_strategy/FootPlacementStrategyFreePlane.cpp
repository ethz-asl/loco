/*
 * FootPlacementStrategyFreePlane.cpp
 *
 *  Created on: Sep 16, 2014
 *      Author: dario
 */

#include "loco/foot_placement_strategy/FootPlacementStrategyFreePlane.hpp"
#include "loco/common/TorsoBase.hpp"
#include "loco/temp_helpers/math.hpp"

namespace loco {

  FootPlacementStrategyFreePlane::FootPlacementStrategyFreePlane(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain) :
      FootPlacementStrategyInvertedPendulum(legs, torso, terrain)
  {
  }


  FootPlacementStrategyFreePlane::~FootPlacementStrategyFreePlane() {

  }


  Position FootPlacementStrategyFreePlane::getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep) {

    //--- Save rotations
    const RotationQuaternion orientationWorldToBaseInWorldFrame = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();
    const RotationQuaternion orientationWorldToHeading = torso_->getMeasuredState().getWorldToHeadingOrientation();
    const RotationQuaternion orientationHeadingToBase = torso_->getMeasuredState().getHeadingToBaseOrientation();
    //---

    //--- Get heading linear velocity expressed in world frame
    LinearVelocity desiredLinearVelocityBaseInHeadingFrame = orientationHeadingToBase.inverseRotate(torso_->getDesiredState().getBaseLinearVelocityInBaseFrame());
    desiredLinearVelocityBaseInHeadingFrame.y() = 0.0;
    desiredLinearVelocityBaseInHeadingFrame.z() = 0.0;
    LinearVelocity desiredLinearVelocityBaseInWorldFrame = orientationWorldToHeading.inverseRotate(desiredLinearVelocityBaseInHeadingFrame);
    //---

    double swingPhase = 1;
    if (leg->isInSwingMode()) {
      swingPhase = leg->getSwingPhase();
    }

    //--- Get default desired positions
    const Position desiredDefaultSteppingPositionHipToFootInHeadingFrame = leg->getProperties().getDesiredDefaultSteppingPositionHipToFootInHeadingFrame();
    const Position defaultPositionHipToFootHoldInWorldFrame = orientationWorldToHeading.inverseRotate(desiredDefaultSteppingPositionHipToFootInHeadingFrame); // todo
    //---

    // inverted pendulum stepping offset
    const Position rRef_CSw = leg->getWorldToHipPositionInWorldFrame();
    LinearVelocity vRef_CSw = ( leg->getHipLinearVelocityInWorldFrame() + orientationWorldToBaseInWorldFrame.inverseRotate(torso_->getMeasuredState().getBaseLinearVelocityInBaseFrame()) )/2.0;

    const Position invertedPendulumStepLocation_CSw = rRef_CSw; // --> hip position in world frame --> end of inverted pendulum
    double invertedPendulumHeight = std::max((rRef_CSw.z() - getHeightOfTerrainInWorldFrame(invertedPendulumStepLocation_CSw)), 0.0);

    LinearVelocity vError_CSw = vRef_CSw - desiredLinearVelocityBaseInWorldFrame;

    const double gravity = torso_->getProperties().getGravity().norm();

    Position invertedPendulumPositionHipToFootHoldInWorldFrame = Position(vError_CSw)*std::sqrt(invertedPendulumHeight/gravity);

    // limit offset from inverted pendulum
  //  const double legLengthMax = 0.5*leg->getProperties().getLegLength();
    invertedPendulumPositionHipToFootHoldInWorldFrame.z() = 0.0;
  //  double desLegLength = rFootHoldOffset_CSw_invertedPendulum.norm();
  //  if (desLegLength != 0.0) {
  //    double boundedLegLength = desLegLength;
  //    boundToRange(&boundedLegLength, -legLengthMax, legLengthMax);
  //    rFootHoldOffset_CSw_invertedPendulum *= boundedLegLength/desLegLength;
  //  }


    // feedforward stepping offset
    //we also need to add a desired-velocity dependent feed forward step length
    double netCOMDisplacementPerStride = desiredLinearVelocityBaseInHeadingFrame.x() * leg->getStanceDuration();
    //this is relative to the hip location, hence the divide by 2.0
    double feedForwardStepLength = netCOMDisplacementPerStride / 2.0;

    Position feedForwardPositionHipToFootHoldInWorldFrame = orientationWorldToHeading.inverseRotate(Position(feedForwardStepLength, 0.0, 0.0));

    Position rFootHoldOffset_CSw_final = defaultPositionHipToFootHoldInWorldFrame + feedForwardPositionHipToFootHoldInWorldFrame + stepFeedbackScale_*invertedPendulumPositionHipToFootHoldInWorldFrame;
    rFootHoldOffset_CSw_final.z() = 0.0;

    const Position positionHipToFootInWorldFrameAtLiftOff = leg->getStateLiftOff()->getFootPositionInWorldFrame()-leg->getStateLiftOff()->getHipPositionInWorldFrame();
    Position rFootOffset_CSw = getCurrentFootPositionFromPredictedFootHoldLocationInWorldFrame(std::min(swingPhase + tinyTimeStep, 1.0),  positionHipToFootInWorldFrameAtLiftOff, rFootHoldOffset_CSw_final, leg);

    Position rFoot_CSw = rRef_CSw + (Position(vRef_CSw)*tinyTimeStep + rFootOffset_CSw);
    positionWorldToFootHoldInWorldFrame_[leg->getId()] = rRef_CSw + (Position(vRef_CSw)*tinyTimeStep + rFootHoldOffset_CSw_final);
    positionWorldToFootHoldInWorldFrame_[leg->getId()].z() = getHeightOfTerrainInWorldFrame(positionWorldToFootHoldInWorldFrame_[leg->getId()]);

    positionWorldToFootHoldInvertedPendulumInWorldFrame_[leg->getId()] =  rRef_CSw + Position(vRef_CSw)*tinyTimeStep + defaultPositionHipToFootHoldInWorldFrame;
    positionWorldToFootHoldInvertedPendulumInWorldFrame_[leg->getId()].z() = getHeightOfTerrainInWorldFrame(positionWorldToFootHoldInvertedPendulumInWorldFrame_[leg->getId()]);

    // to avoid slippage, do not move the foot in the horizontal plane when the leg is still grounded
    if (leg->isGrounded()) {
      rFoot_CSw = leg->getWorldToFootPositionInWorldFrame();
    }

    rFoot_CSw.z() = getHeightOfTerrainInWorldFrame(rFoot_CSw) + swingFootHeightTrajectory_.evaluate(std::min(swingPhase + tinyTimeStep, 1.0));

    return rFoot_CSw;
  }


} /* namespace loco */
