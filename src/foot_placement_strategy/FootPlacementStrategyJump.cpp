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

bool FootPlacementStrategyJump::loadParameters(const TiXmlHandle& handle) {

  return true;
}

bool FootPlacementStrategyJump::initialize(double dt) {
  LegBase* leftForeLeg = legs_->getLeftForeLeg();

  leftForeInitJointPositions_ = leftForeLeg->getMeasuredJointPositions();

  return true;
}

void FootPlacementStrategyJump::advance(double dt) {

  /* Calculate landing positions for legs after doing a jump */
  LegBase* leftForeleg = legs_->getLeftForeLeg();
  LegBase* rightForeleg = legs_->getRightForeLeg();
  LegBase* leftHindleg = legs_->getLeftHindLeg();
  LegBase* rightHindleg = legs_->getRightHindLeg();

  if (!leftForeleg->isGrounded() && !leftHindleg->isGrounded() && !rightHindleg->isGrounded() && !rightForeleg->isGrounded()) {
    leftForeleg->setDesiredJointPositions(leftForeInitJointPositions_);

    rightForeleg->setDesiredJointPositions(leftForeInitJointPositions_);

    leftHindleg->setDesiredJointPositions(-leftForeInitJointPositions_);

    rightHindleg->setDesiredJointPositions(-leftForeInitJointPositions_);
  }
}

}  // namespace loco
