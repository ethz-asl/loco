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

#include "FootPlacementStrategyBase.hpp"
#include "tinyxml.h"
#include <Eigen/Core>

#include "Trajectory.hpp"

#include "kindr/rotations/RotationEigen.hpp"


namespace loco {



//! This class implements a push recovery strategy based on the inverted pendulum.
/*! Each leg computes independently the next foothold location.
 * @ingroup robotTask
 */
class FootPlacementStrategyInvertedPendulum: public FootPlacementStrategyBase {
public:
  typedef kindr::rotations::eigen_impl::RotationQuaternionPD RotationQuaternion;
public:
	FootPlacementStrategyInvertedPendulum();
	virtual ~FootPlacementStrategyInvertedPendulum();


	void setFootLocationAtLiftOff(int iLeg, const Eigen::Vector3d& footLocationAtLiftOffCSw);

	// properties
  void setGravity(double gravity);
  void setSwingFootHeightTrajectory(const Trajectory1D& swingFootHeightTrajectory);
  void setSwingPhase(int iLeg, double swingPhase);
  void setStanceDuration(int iLeg, double stanceDuration);

  void setGroundHeight(int iLeg, double groundHeightCSw);
  void setHipPosition(int iLeg, const Eigen::Vector3d& rHip_CSw);
  void setHipVelocity(int iLeg, const Eigen::Vector3d& vHip_CSw);
  void setBaseVelocity(int iLeg, const Eigen::Vector3d& vBase_CSw);

  void setRotationWorldToBase(const RotationQuaternion& p_BW);



  void setSteppingOffsetToHip(int iLeg, const Eigen::Vector3d& steppingOffsetToHip_CSmb);


  /*! Sets the desired heading speed of the robot
   * @param desiredHeadingSpeed   [m/s]
   */
  void setDesiredHeadingSpeed(double desiredHeadingSpeed);


	/*! Gets the foot position for the swing leg
	 *
	 * @param leg	reference to the leg
	 * @param dt	tiny time step in the future to compute the desired velocities
	 * @return
	 */
	virtual Eigen::Vector3d getFootPositionForSwingLegCSw(int iLeg, double dt);




	void setFeedbackScale(double scale);
public:
	//! and this swing-phase based trajectory is used to control the desired swing foot position (interpolating between initial location of the step, and final target) during swing.
	Trajectory1D stepInterpolationFunction;

	//! this value is used to scale the default feedback contribution for the stepping location
	double stepFeedbackScale_;

	//! foot location at lift-off expressed in world frame
  Eigen::Vector3d footLocationAtLiftOffCSw_[4];

  //! gravitational acceleration (default: 9.81)
  double gravity_;

  //! trajectory of the height of the swing foot above ground over the swing phase
  Trajectory1D swingFootHeightTrajectory_;

  //! desired heading speed
  double desiredHeadingSpeed_;

  //! stance duration for each leg
  double stanceDuration_[4];

  //! current swing phase for each leg  in range [0, 1] (1 if in stance mode)
  double swingPhase_[4];

  //! estimated ground height for each leg in world frame
  double estimatedGroundHeightCSw_[4];

  //! default stepping offset with respect to the hip (only x and y coordinates are considered)
  Eigen::Vector3d steppingOffsetToHip_CSmb_[4];

  //! position of the hip joint expressed in world frame
  Eigen::Vector3d rHip_CSw_[4];

  //! linear velocity of the hip joint expressed in world frame
  Eigen::Vector3d vHip_CSw_[4];

  //! linear velocity of the base expressed in world frame
  Eigen::Vector3d vBase_CSw_;

  //! passive rotation quaternion from world to base frame
  RotationQuaternion p_BW_;



protected:
	double getCoronalComponentOfFootStep(double phase, double initialStepOffset, double stepGuess);
	double getSagittalComponentOfFootStep(double phase, double initialStepOffset, double stepGuess);


	Eigen::Vector3d getCurrentFootPositionFromPredictedFootHoldLocation(double phase, const Eigen::Vector3d& footLocationAtLiftOffCSw, const Eigen::Vector3d& rFootHold_CSw, const RotationQuaternion& p_BW);



};

} // namespace loco
#endif /* LOCO_FOOTPLACEMENTSTRATEGYINVERTEDPENDULUM_HPP_ */
