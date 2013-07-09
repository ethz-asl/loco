/*!
* @file 	Sandbox_Task.hpp
* @author 	Christian Gehring
* @date 	Jan 16, 2011
* @version 	1.0
* @ingroup 	robotTask
* @brief	Standing with Virtual Model at Base
*/

#ifndef RoughTerrain_HPP_
#define RoughTerrain_HPP_

#include "TaskRobotBase.hpp"




// robotUtils
#include "TaskTimer.hpp"
#include "DisturbRobot.hpp"




namespace robotTask {

//! Sandbox Task.
/*! Used to test and play with stuff.
 * @ingroup robotTask
 */
class RoughTerrain:public robotTask::TaskRobotBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW


	/*! Constructor.
	 * @param robotModel	reference to the robot
	 */
	RoughTerrain(robotModel::RobotModel* robotModel);

	//! Destructor.
	~RoughTerrain();

	/*! Adds the task.
	 * @return	true if successful
	 */
	virtual bool add();

	/*! Initializes the task.
	 * @return	true if successful
	 */
	virtual bool init();

	/*! Runs the task.
	 * @return	true if successful
	 */
	virtual bool run();

	/*! Changes the parameters of the task.
	 * Loads a menu where the user can change parameters of this task.
	 * @return	true if successful
	 */
	virtual bool change();
private:


	//! sampling time of process =1/servo_rate
	double time_step_;

	robotUtils::TaskTimer timer_;

	robotUtils::DisturbRobot* disturbRobot_;





};

}

#endif /* Sandbox_HPP_ */
