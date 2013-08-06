/*
 * RoughTerrain_Task.hpp
 *
 *  Created on: Jul 9, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "RoughTerrain_Task.hpp"

// Eigen
#include <Eigen/Geometry>

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
using namespace Eigen;

namespace robotTask {

RoughTerrain::RoughTerrain(RobotModel* robotModel) : TaskRobotBase("RoughTerrain", robotModel)
{
  disturbRobot_ = new DisturbRobot();
  virtualModelController_ = new VirtualModelController(robotModel_);
}

RoughTerrain::~RoughTerrain()
{
  delete disturbRobot_;
  delete virtualModelController_;
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

  VectorP baseDesiredPosition = VectorP::Zero(3); // TODO Why is VectorP dynamic?
  Quaterniond baseDesiredOrientation = Quaterniond::Identity();
  VectorP baseDesiredLinearVelocity = VectorP::Zero(3);
  VectorO baseDesiredAngularVelocity = VectorO::Zero(3);

  virtualModelController_->computeTorques(baseDesiredPosition, baseDesiredOrientation, baseDesiredLinearVelocity, baseDesiredAngularVelocity);

  return true;
}

bool RoughTerrain::change()
{
  int key = 999;
  int jointID = 1;
  int myvalue = 0;
  int ivalue = 0;
  double value;
  Eigen::Vector3d disturbanceForce(1000.0, 100.0, -100.0);
  Eigen::Vector3d disturbanceTorque(0.0, 0.0, 100.0);

  while (true)
  {
    /* show the possibilities */
    cout << "[0]\tExit" << endl;
    cout << "[1]\tDisturb robot" << endl;
    cout << "[2]\tReset simulation" << endl;

    get_int("What to do?", key, &key);

    /* change */
    switch (key)
    {
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

//void RoughTerrain::computeVirtualModelControl()
//{
////  //! Error vector between desired and target
////  Eigen::Vector3d positionError = bodyTargetPosition_ - bodyDesiredPosition_;
////
////  // Make the desired position move towards the target position with a limited speed
////  if (positionError.norm() > baseMaxSpeed_ * getTimeStep())
////  {
////    bodyDesiredPosition_ += positionError.normalized() * baseMaxSpeed_ * getTimeStep();
////  }
////  else
////  {
////    bodyDesiredPosition_ = bodyTargetPosition_;
////  }
//
////  virtualModelController_->computeForce(F_, M_, I_rEst_IB_, I_vEst_B_, rpyEst_BI_,
////                           I_wEst_IB_, bodyDesiredPosition_, I_vDes_B_, rpyDes_BI_,
////                           I_wDes_IB_, I_rVec_B_Cog_);
//}
//
//void RoughTerrain::computeContactForceDistribution()
//{
//
//}

} /* namespace robotTask */
