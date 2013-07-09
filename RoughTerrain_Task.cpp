/*!
* @file 	Sandbox_Task.cpp
* @author 	Christian Gehring
* @date		Jan 16, 2011
* @version 	1.0
* @ingroup 	robotTask
* @brief
*/

#include "RoughTerrain_Task.hpp"

#include "DrawArrow.hpp"
#include "DrawSphere.hpp"
#include "DrawFrame.hpp"
#include "DrawGhost.hpp"

using namespace std;
using namespace robotModel;
using namespace robotTask;
using namespace robotUtils;

//void quatDerviativeToAngularVelocity(SL_quat* q)
//{
//	  double Q[3+1][4+1];
//
//
//	  Q[1][1] = -q->q[2];
//	  Q[1][2] = q->q[1];
//	  Q[1][3] = q->q[4];
//	  Q[1][4] = -q->q[3];
//
//	  Q[2][1] = -q->q[3];
//	  Q[2][2] = -q->q[4];
//	  Q[2][3] = q->q[1];
//	  Q[2][4] = q->q[2];
//
//	  Q[3][1] = -q->q[4];
//	  Q[3][2] = q->q[3];
//	  Q[3][3] = -q->q[2];
//	  Q[3][4] = q->q[1];
//
//	  for (int j=1; j<=N_CART; ++j) {
//	    q->ad[j] = 0.0;
//	    for (int i=1; i<=N_QUAT; ++i) {
//	      q->ad[j] += 2*Q[j][i]*q->qd[i];
//	    }
//	  }
//}
//

//void quatDerviativeToAngularVelocity(SL_quat* q)
//{
//	  double Q[3+1][4+1];
//
//
//	  Q[1][1] = -q->q[2];
//	  Q[1][2] = q->q[1];
//	  Q[1][3] = -q->q[4];
//	  Q[1][4] = q->q[3];
//
//	  Q[2][1] = -q->q[3];
//	  Q[2][2] = q->q[4];
//	  Q[2][3] = q->q[1];
//	  Q[2][4] = -q->q[2];
//
//	  Q[3][1] = -q->q[4];
//	  Q[3][2] = -q->q[3];
//	  Q[3][3] = q->q[2];
//	  Q[3][4] = q->q[1];
//
//	  for (int j=1; j<=N_CART; ++j) {
//	    q->ad[j] = 0.0;
//	    for (int i=1; i<=N_QUAT; ++i) {
//	      q->ad[j] += 2*Q[j][i]*q->qd[i];
//	    }
//	  }
//}
//


RoughTerrain::RoughTerrain(RobotModel* robotModel)
:TaskRobotBase("RoughTerrain",	// name of task
		  robotModel)	// reference to robot model
{
	disturbRobot_ = new robotUtils::DisturbRobot;

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


//	Vector aa;
//	 aa = my_vector(1,N_CART);
//	 // aa indexing according RPY (Roll, Pitch, Yaw)
//
//	 aa[1] = 4.3;
//	 aa[2] = 6.05;
//	 aa[3] = 9.734;
//
//
//	SL_quat q;
//	q.ad[_X_] = 9.2;
//	q.ad[_Y_] = 3.1;
//	q.ad[_Z_] = 7.3;

//	q.q[_Q0_] = 1.0;
//	q.q[_Q1_] = 0.0;
//	q.q[_Q2_] = 0.0;
//	q.q[_Q3_] = 0.0;

//	 eulerToQuat(aa, &q);

//	q.dq[_Q0_] = 1.0;
//	q.dq[_Q1_] = 1.0;
//	q.dq[_Q2_] = 1.0;
//	q.dq[_Q3_] = 1.0;

//	 printf("q: %lf %lf %lf %lf\n", q.q[_Q0_], q.q[_Q1_], q.q[_Q2_], q.q[_Q3_]);
//	printf("ad: %lf %lf %lf\n",q.ad[_X_], q.ad[_Y_], q.ad[_Z_]);
//	quatDerivatives(&q);
//	printf("dq: %lf %lf %lf %lf\n", q.qd[_Q0_], q.qd[_Q1_], q.qd[_Q2_], q.qd[_Q3_]);
//	quatDerviativeToAngularVelocity(&q);
//	printf("ad: %lf %lf %lf\n",q.ad[_X_], q.ad[_Y_], q.ad[_Z_]);


//
//
//
////	Eigen::Vector3d p;
////	p << 0,0,0;
////	traj.addKnot(0,p);
////
////	p << 1,10,100;
////	traj.addKnot(1,p);
////	p << 1,20,200;
////	traj.addKnot(2,p);
////
////	std::cout <<  "0:\n" << traj.evaluate_catmull_rom(0);
////	std::cout <<  "\n1.5:\n" << traj.evaluate_catmull_rom(1.5);
////
////	std::cout <<  "0:\n" << traj.evaluate_linear(0);
////	std::cout <<  "\n1.5:\n" << traj.evaluate_linear(1.5);
////	Trajectory1D test;
////	test.saveCatmullRomToFile(100,std::string("trajtest.txt"));
//	robotModel::VectorQb qb = robotModel::VectorQb::Zero();
//	robotModel::VectorQj qj = robotModel::VectorQj::Zero();
//	qb(robotModel::qZ) = 0.6;
//	//freezeBase(true);
//
//
//
//
//
//	timer_.startTimer();
////	robotUtils::DrawArrow drawArrow;
////	drawArrow.drawArrowInWFrame(1, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0.5,0.5,0.5));
//	robotModel::VectorQ q = robotModel_->q().getQ();
//	q(2)+=0.2;
//	q(0)= 0.2*sin(0.1*2*M_PI*timer_.getElapsedTime());
//	q(1)= 0.2*cos(0.1*2*M_PI*timer_.getElapsedTime());
//	robotUtils::DrawGhost::draw(q);
//
//	robotUtils::DrawFrame::drawInWorldFrame(0, Eigen::Vector3d(0.1,0.0,0.7), Eigen::Vector3d(0.0,0,0.0), robotUtils::DrawFrame::BLUE,1);
//	robotUtils::DrawFrame::drawInWorldFrame(1, Eigen::Vector3d(0.2,0.0,0.7), Eigen::Vector3d(0,0,0), robotUtils::DrawFrame::RED,1);
//	robotUtils::DrawFrame::drawInBaseFrame(0, Eigen::Vector3d(0.3,0.0,0.7), Eigen::Vector3d(0,0,0.0), robotUtils::DrawFrame::GREEN,1);
//	robotUtils::DrawFrame::drawInBaseFrame(1, Eigen::Vector3d(0.4,0.0,0.7), Eigen::Vector3d(0,0,0.0), robotUtils::DrawFrame::YELLOW,1);
//
//	robotUtils::DrawArrow::drawInWorldFrame(0, Eigen::Vector3d(0.1,0.0,0.1), Eigen::Vector3d(0.0,0,0.1), robotUtils::DrawArrow::BLUE,0.5);
//	robotUtils::DrawArrow::drawInWorldFrame(1, Eigen::Vector3d(0.2,0.0,0.2), Eigen::Vector3d(0,0,0.2), robotUtils::DrawArrow::RED,0.5);
//	robotUtils::DrawArrow::drawInBaseFrame(0, Eigen::Vector3d(0.3,0.0,0.3), Eigen::Vector3d(0,0,0.3), robotUtils::DrawArrow::GREEN,0.5);
//	robotUtils::DrawArrow::drawInBaseFrame(1, Eigen::Vector3d(0.4,0.0,0.4), Eigen::Vector3d(0,0,0.4), robotUtils::DrawArrow::YELLOW,1);
//
//	robotUtils::DrawSphere::drawInWorldFrame(0, Eigen::Vector3d(0.1,0.0,1.0), 0.01, robotUtils::DrawSphere::BLUE,0.5);
//	robotUtils::DrawSphere::drawInWorldFrame(1, Eigen::Vector3d(0.2,0.0,1.0), 0.02, robotUtils::DrawSphere::RED,0.5);
//	robotUtils::DrawSphere::drawInBaseFrame(0, Eigen::Vector3d(0.3,0.0,1.0), 0.03, robotUtils::DrawSphere::GREEN,0.5);
//	robotUtils::DrawSphere::drawInBaseFrame(1, Eigen::Vector3d(0.4,0.0,1.0), 0.04, robotUtils::DrawSphere::YELLOW,1);

	return true;
}
bool RoughTerrain::run()
{
//
//	float ball[N_CART+1];
//	ball[_X_] = 0.2*sin(0.1*2*M_PI*timer_.getElapsedTime());
//	ball[_Y_] = 0.2*cos(0.1*2*M_PI*timer_.getElapsedTime());
//	ball[_Z_] = 0.2;
//	sendUserGraphics("redball",&ball[_X_],N_CART*sizeof(float));

//	resetSim.reset();

//	robotUtils::DrawArrow::drawArrowInWFrame(1, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0.2*sin(0.1*2*M_PI*timer_.getElapsedTime()),0.2*cos(0.1*2*M_PI*timer_.getElapsedTime()),0.2));

//	robotModel::VectorQ q = robotModel_->q().getQ();
//	q(2)+=0.2;
//	q(0)= 0.2*sin(0.1*2*M_PI*timer_.getElapsedTime());
//	q(1)= 0.2*cos(0.1*2*M_PI*timer_.getElapsedTime());
//	robotUtils::DrawGhost::draw(q);
//

//	Eigen::Vector3d pos = robotModel_->kin().getJacobianTByLeg_Base2Foot_CSw(0)->getPos();
	Eigen::Vector3d pos2 = robotModel_->kin()(JR_Base2World)->getA()*robotModel_->kin().getJacobianTByLeg_Base2Hip_CSmb(0)->getPos();
//
//	cout << "error pos\n" << pos-pos2 << endl;

//	disturbRobot_->disturbOverInterval();
//	drawarrow_.drawArrowInMbFrame(0, {0,0,0},{0,0.1,0.39});
//	drawarrow_.drawArrowInMbFrame(3, {0,0,0},{0,0,-0.39});
//	drawsphere_.drawSphereInMbFrame(0,{0.1,0,0.0},0.005);
//
//	robotModel::VectorQb mbForce;
//
//	double z = robotModel_->q().getQb()(2);
//	double dz = robotModel_->q().getdQb()(2);
//	double feedBack = 2000*(0.39-z) + 100*(-dz);
//	robotModel::VectorAct jointTorques;
//	robotModel::VectorActM mode;
//	mode.setConstant(AM_Torque);
//
//	mbForce << 0,0,220+feedBack,0,0,0;
//	distributeForce_->distributeMainBodyForcesCSw(mbForce,jointTorques);
//	robotModel_->act().setTau(jointTorques);
//	robotModel_->act().setMode(mode);
//	robotModel_->act().setCommands();



	return true;
}

bool RoughTerrain::change()
{



	int key = 999;
	int jointID = 1;
	int myvalue = 0;
	int ivalue = 0;
	double value;
	Eigen::Vector3d force(0.0,0.0,-100.0);
	Eigen::Vector3d torque(0.0,0.0, 10.0);
	Eigen::Vector4d vfModifier;
	robotModel::VectorAct jointTorques;
		robotModel::VectorAct jointTorques4;
		robotModel::VectorQb mbForce;

	while (true) {
		/* show the possibilities */
		cout << "[0]\tExit" << endl;
		cout << "[1]\tbla bla" << endl;
		cout << "[2]\tbla bla" << endl;
		cout << "[3]\tbla bla" << endl;
		cout << "[4]\treset simulation" << endl;

		get_int("What to do?",key,&key);

		/* change */
		switch (key) {
		case 0:
			/* exit */
			return true;
			break;
		case 1:
			/* bla bla */
			disturbRobot_->setDisturbanceToZero();
			disturbRobot_->addForceCSmbToMainBody(force);
			disturbRobot_->disturbOverInterval(1.0);
			break;
		case 2:
			/* bla bla */
			disturbRobot_->setDisturbanceToZero();
			disturbRobot_->addTorqueCSmbToMainBody(torque);
			disturbRobot_->disturbOverInterval(1.0);
			break;
		case 3:
			/* bla bla */
			disturbRobot_->setDisturbanceToZero();
			disturbRobot_->addForceCSmbToMainBody(force);
			disturbRobot_->addTorqueCSmbToMainBody(torque);
			disturbRobot_->disturbOverInterval(1.0);
			break;
		case 4:
			/* reset simulation */
			{
				robotUtils::DrawSphere::hideInWorldFrame(0);
			}
			break;
		case 5:
			/* reset simulation */
			{
				robotUtils::DrawSphere::hideInWorldFrame(1);
			}
			break;
		case 6:
			/* reset simulation */
			{

				robotUtils::DrawSphere::hideInBaseFrame(0);
//				robotUtils::DrawArrow drawArrow;
//				robotUtils::DrawArrow::drawInWorldFrame(1, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0.5,-0.5,0.5), robotUtils::DrawArrow::BLUE);
//				robotUtils::DrawArrow::drawArrowInWFrame(2, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0.5,-0.5,0.5));
//				drawArrow.drawArrowInWFrame(2, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0.5,-0.5,0.5));

			}
			break;
			case 7:
			{
				robotUtils::DrawSphere::hideInBaseFrame(1);
//				robotUtils::DrawArrow::drawInBaseFrame(1, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0.5,0,0.5), robotUtils::DrawArrow::BLUE,1);
			}
			break;
		case 8:
				robotUtils::DrawSphere::drawInBaseFrame(0, Eigen::Vector3d(0.2,0,0), 0.1, robotUtils::DrawArrow::BLUE, 0.5);
				robotUtils::DrawArrow::drawInBaseFrame(2, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0.0,0,0.1), robotUtils::DrawArrow::RED,1);
				break;
		case 9:
				robotUtils::DrawGhost::hide();
				break;
		default:
			key = 0;
			break;
		}
	}
	return true;
}

