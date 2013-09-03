/*
 * RoughTerrain_Task.hpp
 *
 *  Created on: Jul 9, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "RoughTerrain_Task.hpp"
#include <Eigen/Geometry>
#include "DrawArrow.hpp"
#include "DrawSphere.hpp"
#include "DrawFrame.hpp"
#include "DrawGhost.hpp"
#include "Rotations.hpp"
#include "toMove.hpp"

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

  disturbanceTime_ = 0.1;
  disturbanceForceMagnitude_ = 0.5 * 5000.0;
  disturbanceTorqueMagnitude_ = 0.5* 1000.0;
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
  virtualModelController_->addToLogger();
  virtualModelController_->loadParameters();
  return true;
}

bool RoughTerrain::run()
{
  //! Desired base position expressed in inertial frame.
  VectorP baseDesiredPosition(0.0, 0.0, 0.425);
  //! Desired base orientation (quaternion) w.r.t. inertial frame.
  Quaterniond baseDesiredOrientation = Quaterniond::Identity();
  baseDesiredOrientation = Rotations::yawPitchRollToQuaternion(1.0, 0.0, 0.0);
  //! Desired base linear velocity expressed in inertial frame.
  VectorP baseDesiredLinearVelocity = VectorP::Zero(3);
  //! Desired base angular velocity expressed w.r.t. inertial frame.
  VectorO baseDesiredAngularVelocity = VectorO::Zero(3);

  if(getTime() > 5.0)
    virtualModelController_->changeLegLoad(Legs::RIGHT_FRONT, 0.1 + (1.0 + sin(4.0 * getTime())) / 2.0);
  virtualModelController_->computeTorques(baseDesiredPosition, baseDesiredOrientation, baseDesiredLinearVelocity, baseDesiredAngularVelocity);

  VectorActM desJointModes;
  VectorAct desJointPositions, desJointVelocities, desJointTorques;
  virtualModelController_->packDesiredJointSetpoints(desJointModes, desJointPositions, desJointVelocities, desJointTorques);

  robotModel_->act().setMode(desJointModes);
  robotModel_->act().setTau(desJointTorques);

  return true;
}

bool RoughTerrain::change()
{
  int key = 999;
  int jointID = 1;
  int myvalue = 0;
  int ivalue = 0;
  double value;

  while (true)
  {
    /* show the possibilities */
    cout << "[0]\tExit" << endl;
    cout << "[1]\tDisturb randomly" << endl;
    cout << "[2]\tDisturb with force in x" << endl;
    cout << "[3]\tDisturb with force in y" << endl;
    cout << "[4]\tDisturb with force in z" << endl;
    cout << "[5]\tDisturb with force in -z" << endl;
    cout << "[6]\tDisturb with torque around x" << endl;
    cout << "[7]\tDisturb with torque around y" << endl;
    cout << "[8]\tDisturb with torque around z" << endl;
    cout << "[9]\tReset simulation" << endl;

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
        disturbRobot();
        break;
      case 2:
        /* disturbRobot */
        disturbRobot(FORCE, Vector3i(1, 0, 0));
        break;
      case 3:
        /* disturbRobot */
        disturbRobot(FORCE, Vector3i(0, 1, 0));
        break;
      case 4:
        /* disturbRobot */
        disturbRobot(FORCE, Vector3i(0, 0, 1));
        break;
      case 5:
        /* disturbRobot */
        disturbRobot(FORCE, Vector3i(0, 0, -1));
        break;
      case 6:
        /* disturbRobot */
        disturbRobot(TORQUE, Vector3i(1, 0, 0));
        break;
      case 7:
        /* disturbRobot */
        disturbRobot(TORQUE, Vector3i(0, 1, 0));
        break;
      case 8:
        /* disturbRobot */
        disturbRobot(TORQUE, Vector3i(0, 0, 1));
        break;
      case 9:
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

bool RoughTerrain::disturbRobot(DisturbanceType disturbanceType,
                                Eigen::Vector3i disturbanceDirection)
{
  disturbRobot_->setDisturbanceToZero();

  if (disturbanceType == FORCE)
  {
    Vector3d disturbanceForce = disturbanceForceMagnitude_ * disturbanceDirection.cast<double>().normalized();
    disturbRobot_->addForceCSmbToMainBody(disturbanceForce);
  }
  if (disturbanceType == TORQUE)
  {
    Vector3d disturbanceTorque = disturbanceTorqueMagnitude_ * disturbanceDirection.cast<double>().normalized();
    disturbRobot_->addTorqueCSmbToMainBody(disturbanceTorque);
  }

  disturbRobot_->disturbOverInterval(disturbanceTime_);
}

bool RoughTerrain::disturbRobot() // TODO Maybe port this to the DisturbRobot class, as it might be helpful in other situations
{
  disturbRobot_->setDisturbanceToZero();

  Vector3d disturbanceForce = disturbanceForceMagnitude_ * Vector3d::Random();
  disturbRobot_->addForceCSmbToMainBody(disturbanceForce);

  Vector3d disturbanceTorque = disturbanceTorqueMagnitude_ * Vector3d::Random();
  disturbRobot_->addTorqueCSmbToMainBody(disturbanceTorque);

  disturbRobot_->disturbOverInterval(disturbanceTime_); // TODO Check if different timings have influence
}

} /* namespace robotTask */
