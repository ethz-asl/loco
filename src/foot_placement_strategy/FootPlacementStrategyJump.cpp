/*!
 * @file   FootPlacementStrategyJump.cpp
 * @author wko
 * @date   Jun 6, 2014
 * @version  1.0
 * @ingroup  robotTask
 * @brief
 */

#include "loco/foot_placement_strategy/FootPlacementStrategyJump.hpp"
#include "loco/common/TorsoBase.hpp"

#include "loco/temp_helpers/math.hpp"

namespace loco {

FootPlacementStrategyJump::FootPlacementStrategyJump(
    LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain)
    : FootPlacementStrategyBase(),
      legs_(legs),
      torso_(torso),
      terrain_(terrain) {

}

FootPlacementStrategyJump::~FootPlacementStrategyJump() {

}

// TODO: See if we need to introduce parameters to calculate landing positions for legs. For now hardcoded, so no parameters needed yet.
bool FootPlacementStrategyJump::loadParameters(const TiXmlHandle& handle) {

  return true;
}

bool FootPlacementStrategyJump::initialize(double dt) {
  return true;
}

void FootPlacementStrategyJump::advance(double dt) {

  /* Calculate landing positions for legs after doing a jump */
  //TODO Actually calculate the positions instead of using hardcoded positions
  const Position leftForeLandingPosition(0.3, 0.25, -0.42);
  const Position leftHindLandingPosition(-0.3, 0.25, -0.42);
  const Position rightForeLandingPosition(0.3, -0.25, -0.42);
  const Position rightHindLandingPosition(-0.3, -0.25, -0.42);

  LegBase* leftForeleg = legs_->getLeftForeLeg();
  LegBase* rightForeleg = legs_->getRightForeLeg();
  LegBase* leftHindleg = legs_->getLeftHindLeg();
  LegBase* rightHindleg = legs_->getRightHindLeg();

  if ((torso_->getMeasuredState()).getBaseLinearVelocityInBaseFrame().z() > 0.5) {
    if (leftForeleg->isInSwingMode() && !leftForeleg->isGrounded()) {
      leftForeleg->setDesiredJointPositions(
          leftForeleg->getJointPositionsFromBaseToFootPositionInBaseFrame(
              leftForeLandingPosition));
    }
    if (rightForeleg->isInSwingMode() && !rightForeleg->isGrounded()) {
      rightForeleg->setDesiredJointPositions(
          rightForeleg->getJointPositionsFromBaseToFootPositionInBaseFrame(
              rightForeLandingPosition));
    }
    if (leftHindleg->isInSwingMode() && !leftHindleg->isGrounded()) {
      leftHindleg->setDesiredJointPositions(
          leftHindleg->getJointPositionsFromBaseToFootPositionInBaseFrame(
              leftHindLandingPosition));
    }
    if (rightHindleg->isInSwingMode() && !rightHindleg->isGrounded()) {
      rightHindleg->setDesiredJointPositions(
          rightHindleg->getJointPositionsFromBaseToFootPositionInBaseFrame(
              rightHindLandingPosition));
    }
  }
}

}  // namespace loco
