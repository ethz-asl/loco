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
#include "robotUtils/loggers/Logger.hpp"
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

 double heightByTrajectory_[4];
 Position invertedPendulumPositionHipToFootHoldInWorldFrame_[4];
public:
 Position positionWorldToFootHoldInWorldFrame_[4];
 Position positionWorldToFootHoldInvertedPendulumInWorldFrame_[4];
  FootPlacementStrategyInvertedPendulum(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain);
  virtual ~FootPlacementStrategyInvertedPendulum();

  virtual bool loadParameters(const TiXmlHandle& handle);
  virtual bool initialize(double dt);
  virtual void advance(double dt);
  /*! Computes an interpolated version of the two controllers passed in as parameters.
  *  If t is 0, the current setting is set to footPlacementStrategy1, 1 -> footPlacementStrategy2, and values in between
  *  correspond to interpolated parameter set.
  * @param footPlacementStrategy1
  * @param footPlacementStrategy2
  * @param t interpolation parameter
  * @returns true if successful
  */
  virtual bool setToInterpolated(const FootPlacementStrategyBase& footPlacementStrategy1, const FootPlacementStrategyBase& footPlacementStrategy2, double t);
  const LegGroup& getLegs() const;

  const Position& getPositionWorldToDesiredFootHoldInWorldFrame(LegBase* leg) const;
public:
  //! Reference to the legs
  LegGroup* legs_;
  //! Reference to the torso
  TorsoBase* torso_;
  //! Reference to the terrain
  loco::TerrainModelBase* terrain_;

  void setFootTrajectory(LegBase* leg);
  void regainContact(LegBase* leg, double dt);

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



	/*! load foot height trajectory from XML
	 * @param hTrajectory handle
	 * @return true if successful
	 */
  bool loadHeightTrajectory(const TiXmlHandle &hTrajectory);

  bool interpolateHeightTrajectory(rbf::BoundedRBF1D& interpolatedTrajectory, const rbf::BoundedRBF1D& trajectory1, const rbf::BoundedRBF1D& trajectory2, double t);
};

} // namespace loco
#endif /* LOCO_FOOTPLACEMENTSTRATEGYINVERTEDPENDULUM_HPP_ */
