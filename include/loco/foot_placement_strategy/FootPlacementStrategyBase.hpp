/*!
* @file 	FootPlacementStrategyBase.hpp
* @author 	Christian Gehring, Stelian Coros
* @date		  Sep 7, 2012
* @version 	1.0
* @ingroup
* @brief
*/

#ifndef LOCO_FOOTPLACEMENTSTRATEGYBASE_HPP_
#define LOCO_FOOTPLACEMENTSTRATEGYBASE_HPP_

#include <Eigen/Core>

namespace loco {

//! Base class for foot placement algorithms
/*! Derive this class to implement different control algorithms
 * @ingroup robotTask
 */
class FootPlacementStrategyBase {
public:

	/*! Constructor
	 */
	FootPlacementStrategyBase();

	/*! Destructor
	 */
	virtual ~FootPlacementStrategyBase();


	/*! Gets the foot position for the swing leg expressed in world frame
	 * This function is invoked to compute the joint angles of the swing leg by inverse kinematics
	 *
	 * @param iLeg  index of the leg {0, 1, 2, 3}
	 * @param dt	tiny time step in the future to compute the desired velocities
	 * @return  desired foot position expressed in world coordinates
	 */
	virtual Eigen::Vector3d getFootPositionForSwingLegCSw(int iLeg, double dt) = 0;

	virtual void advance(double dt) = 0;
};

} // namespace loco

#endif /* LOCO_FOOTPLACEMENTSTRATEGYBASE_HPP_ */
