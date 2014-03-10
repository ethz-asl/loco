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


#include "loco/common/TypeDefs.hpp"

#include <Eigen/Core>
#include <tinyxml.h>

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
	 * @param tinyTimeStep	tiny time step in the future to compute the desired velocities
	 * @return  desired foot position expressed in world coordinates
	 */
	virtual Position getDesiredWorldToFootPositionInWorldFrame(int iLeg, double tinyTimeStep) = 0;

  virtual bool loadParameters(const TiXmlHandle& handle) = 0;
  virtual bool initialize(double dt) = 0;

	virtual void advance(double dt) = 0;
};

} // namespace loco

#endif /* LOCO_FOOTPLACEMENTSTRATEGYBASE_HPP_ */
