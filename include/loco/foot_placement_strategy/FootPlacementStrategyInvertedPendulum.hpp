/*!
* @file 	FootPlacementStrategyInvertedPendulum.hpp
* @author 	Christian Gehring, Stelian Coros
* @date		Sep 7, 2012
* @version 	1.0
* @ingroup 	robotTask
* @brief
*/

#ifndef LOCO_FOOTPLACEMENTSTRATEGYINVERTEDPENDULUM_HPP_
#define LOCO_FOOTPLACEMENTSTRATEGYINVERTEDPENDULUM_HPP_

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



//! This class implements a push recovery strategy based on the inverted pendulum.
/*! Each leg computes independently the next foothold location.
 * @ingroup robotTask
 */
class FootPlacementStrategyInvertedPendulum: public FootPlacementStrategyBase {
 public:
// typedef Trajectory1D SwingFootHeightTrajectory;
 typedef  rbf::BoundedRBF1D SwingFootHeightTrajectory;

public:
	FootPlacementStrategyInvertedPendulum(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain);
	virtual ~FootPlacementStrategyInvertedPendulum();

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

	double getLateralComponentOfFootStep(double phase, double initialStepOffset, double stepGuess);
	double getHeadingComponentOfFootStep(double phase, double initialStepOffset, double stepGuess);


	/*! Computes current desired foot position by interpolating between the predicted and last foothold depending on the swing phase
	 *
	 * @param swingPhase                                           interpolation parameter
	 * @param positionWorldToFootAtLiftOffInWorldFrame             foot location at lift-off
	 * @param positionWorldToFootAtNextTouchDownInWorldFrame       foot location at next touch-down
	 * @return desired foot position in World frame
	 */
	Position getCurrentFootPositionFromPredictedFootHoldLocationInWorldFrame(double swingPhase, const loco::Position& positionWorldToFootAtLiftOffInWorldFrame, const loco::Position& positionWorldToFootAtNextTouchDownInWorldFrame);

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

} // namespace loco
#endif /* LOCO_FOOTPLACEMENTSTRATEGYINVERTEDPENDULUM_HPP_ */
