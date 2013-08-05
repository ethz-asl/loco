/*!
* @file 	RoughTerrain_Task.cpp
* @author 	Peter Fankhauser
* @date		Jul 9, 2013
* @version 	1.0
* @ingroup 	robotTask
* @brief	Rough Terrain Task
*/

#include "RoughTerrain_Task.hpp"

// robotUtils
#include "DrawArrow.hpp"
#include "DrawSphere.hpp"
#include "DrawFrame.hpp"
#include "DrawGhost.hpp"

using namespace std;
using namespace robotModel;
using namespace robotTask;
using namespace robotController;
using namespace robotUtils;

RoughTerrain::RoughTerrain(RobotModel* robotModel)
:TaskRobotBase("RoughTerrain",	// name of task
               robotModel)	// reference to robot model
{
	disturbRobot_ = new robotUtils::DisturbRobot;
	virtualModelController_ = new robotController::VmcPp_Ctrl(robotModel_);
}

RoughTerrain::~RoughTerrain()
{
	delete disturbRobot_;
}

bool RoughTerrain::add()
{
	return true;
}

bool RoughTerrain::init()
{
	return true;
}
bool RoughTerrain::run()
{
	return true;
}

bool RoughTerrain::change()
{
	int key = 999;
	int jointID = 1;
	int myvalue = 0;
	int ivalue = 0;
	double value;
	Eigen::Vector3d disturbanceForce(1000.0,100.0,-100.0);
  Eigen::Vector3d disturbanceTorque(0.0,0.0, 100.0);

	while (true) {
		/* show the possibilities */
		cout << "[0]\tExit" << endl;
		cout << "[1]\tDisturb robot" << endl;
		cout << "[2]\tReset simulation" << endl;

		get_int("What to do?",key,&key);

		/* change */
		switch (key) {
		case 0:
			/* exit */
			return true;
			break;
		case 1:
			/* disturbRobot */
			disturbRobot_->setDisturbanceToZero();
			disturbRobot_->addForceCSmbToMainBody(disturbanceForce);
      disturbRobot_->addTorqueCSmbToMainBody(disturbanceTorque);
			disturbRobot_->disturbOverInterval(1.0);
			break;
		case 3:
			/* Reset simulation */
		  resetSimulation();
			break;
		default:
			key = 0;
			break;
		}
	}

	return true;
}
