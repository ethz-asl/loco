/*!
 * @file   FootPlacementStrategyJump.hpp
 * @author wko
 * @date   Jun 6, 2014
 * @version  1.0
 * @ingroup  robotTask
 * @brief
 */

#ifndef LOCO_FOOTPLACEMENTSTRATEGYJUMP_HPP_
#define LOCO_FOOTPLACEMENTSTRATEGYJUMP_HPP_

#include "loco/common/TypeDefs.hpp"
#include "loco/common/TorsoBase.hpp"
#include "loco/common/LegGroup.hpp"

#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"

#include "tinyxml.h"
#include <Eigen/Core>

#include "loco/temp_helpers/Trajectory.hpp"

#include "kindr/rotations/RotationEigen.hpp"
#include "Logger.hpp"
#include "loco/common/TerrainModelBase.hpp"

//#include "PeriodicRBF1DC3.hpp"
//#include "PeriodicRBF1DC1.hpp"
#include "BoundedRBF1D.hpp"

namespace loco {

/*!
 * @ingroup robotTask
 */
class FootPlacementStrategyJump : public FootPlacementStrategyBase {
 public:
  typedef rbf::BoundedRBF1D SwingFootHeightTrajectory;

 public:
  Position positionWorldToFootHoldInWorldFrame_[4];
  Position positionWorldToFootHoldJumpInWorldFrame_[4];

  FootPlacementStrategyJump(LegGroup* legs, TorsoBase* torso,
                            loco::TerrainModelBase* terrain);
  virtual ~FootPlacementStrategyJump();

  virtual bool loadParameters(const TiXmlHandle& handle);
  virtual bool initialize(double dt);
  virtual void advance(double dt);

 public:
  //! Reference to the legs
  LegGroup* legs_;
  //! Reference to the torso
  TorsoBase* torso_;
  //! Reference to the terrain
  loco::TerrainModelBase* terrain_;

  //! and this swing-phase based trajectory is used to control the desired swing foot position (interpolating between initial location of the step, and final target) during swing.
  Trajectory1D stepInterpolationFunction_;

  //! this value is used to scale the default feedback contribution for the stepping location
  double stepFeedbackScale_;

  //! trajectory of the height of the swing foot above ground over the swing phase
  SwingFootHeightTrajectory swingFootHeightTrajectory_;

 protected:
  /*! Gets the foot position for the swing leg
   *
   * @param leg reference to the leg
   * @param tinyTimeStep  tiny time step in the future to compute the desired velocities
   * @return
   */
  virtual Position getDesiredWorldToFootPositionInWorldFrame(LegBase* leg, double tinyTimeStep);

	double getLateralComponentOfFootStep(double phase, double initialStepOffset, double stepGuess, LegBase* leg);
	double getHeadingComponentOfFootStep(double phase, double initialStepOffset, double stepGuess, LegBase* leg);


	/*! Computes current desired foot position by interpolating between the predicted and last foothold depending on the swing phase
	 *
	 * @param swingPhase                                           interpolation parameter
	 * @param positionWorldToFootAtLiftOffInWorldFrame             foot location at lift-off
	 * @param positionWorldToFootAtNextTouchDownInWorldFrame       foot location at next touch-down
	 * @return desired foot position in World frame
	 */
	Position getCurrentFootPositionFromPredictedFootHoldLocationInWorldFrame(double swingPhase, const loco::Position& positionWorldToFootAtLiftOffInWorldFrame, const loco::Position& positionWorldToFootAtNextTouchDownInWorldFrame, LegBase* leg);
  /*! Gets the height of the terrain in world frame at a certain location
   * @param position  location
   * @return  height of the terrain
   */
  double getHeightOfTerrainInWorldFrame(const loco::Position& position);

  /*! load foot height trajectory from XML
   * @param hTrajectory handle
   * @return true if successful
   */
  bool loadHeightTrajectory(const TiXmlHandle &hTrajectory);
};

}  // namespace loco
#endif /* LOCO_FOOTPLACEMENTSTRATEGYJUMP_HPP_ */
