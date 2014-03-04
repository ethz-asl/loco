/*!
* @file     LocomotionControllerDynamicGait.cpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"
#include "Rotations.hpp"

namespace loco {

LocomotionControllerDynamicGait::LocomotionControllerDynamicGait(LegGroup* legs, TorsoBase* torso, robotModel::RobotModel* robotModel,
                                                                 robotTerrain::TerrainBase* terrain, LimbCoordinatorBase* limbCoordinator,
                                                                 FootPlacementStrategyBase* footPlacementStrategy, TorsoControlBase* baseController,
                                                                 VirtualModelController* virtualModelController, ContactForceDistributionBase* contactForceDistribution) :
    LocomotionControllerBase(),
    legs_(legs),
    torso_(torso),
    robotModel_(robotModel),
    terrain_(terrain),
    limbCoordinator_(limbCoordinator),
    footPlacementStrategy_(footPlacementStrategy),
    torsoController_(baseController),
    virtualModelController_(virtualModelController),
    contactForceDistribution_(contactForceDistribution)
{

}

LocomotionControllerDynamicGait::~LocomotionControllerDynamicGait() {

}

bool LocomotionControllerDynamicGait::initialize(double dt)
{
  for (auto leg : *legs_) {
    leg->advance(dt);
    //  std::cout << *leg << std::endl;
  }
  torso_->advance(dt);


  virtualModelController_->loadParameters();
  contactForceDistribution_->loadParameters();



  torsoController_->initialize(dt);
  return true;
}

void LocomotionControllerDynamicGait::advance(double dt) {


  for (auto leg : *legs_) {
    leg->advance(dt);
    //  std::cout << *leg << std::endl;
  }
  torso_->advance(dt);
  limbCoordinator_->advance(dt);



  for (auto leg : *legs_) {
    const double swingPhase = leg->getSwingPhase();
    if ((swingPhase >= 0 && swingPhase <= 0.5) && leg->isInStanceMode()) {
      // possible lift-off
      leg->getStateLiftOff()->setFootPositionInWorldFrame(leg->getWorldToFootPositionInWorldFrame()); // or base2foot?
      leg->getStateLiftOff()->setHipPositionInWorldFrame(leg->getWorldToHipPositionInWorldFrame());
    }
  }
  footPlacementStrategy_->advance(dt);
  int iLeg = 0;
  for (auto leg : *legs_) {
    const Position positionWorldToFootInWorldFrame = footPlacementStrategy_->getDesiredWorldToFootPositionInWorldFrame(iLeg, 0.0);
    const Position positionBaseToFootInWorldFrame = positionWorldToFootInWorldFrame - torso_->getMeasuredState().getWorldToBasePositionInWorldFrame();
    const Position positionBaseToFootInBaseFrame  = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame().rotate(positionBaseToFootInWorldFrame);
    leg->setDesiredJointPositions(leg->getJointPositionsFromBaseToFootPositionInBaseFrame(positionBaseToFootInBaseFrame));
    iLeg++;
  }

  torsoController_->advance(dt);



  //! Desired base position expressed in inertial frame.
//  robotModel::VectorP baseDesiredPosition(0.0, 0.0, 0.475);
  robotModel::VectorP baseDesiredPosition(0.0, 0.0, 0.42);
  //! Desired base orientation (quaternion) w.r.t. inertial frame.
  Eigen::Quaterniond baseDesiredOrientation = Eigen::Quaterniond::Identity();
  baseDesiredOrientation = robotUtils::Rotations::yawPitchRollToQuaternion(0.0, 0.0, 0.0);
  //! Desired base linear velocity expressed in inertial frame.
  robotModel::VectorP baseDesiredLinearVelocity = robotModel::VectorP::Zero(3);
  //! Desired base angular velocity expressed w.r.t. inertial frame.
  robotModel::VectorO baseDesiredAngularVelocity = robotModel::VectorO::Zero(3);

  virtualModelController_->computeTorques(baseDesiredPosition, baseDesiredOrientation, baseDesiredLinearVelocity, baseDesiredAngularVelocity);

  robotModel::VectorActM desJointModes;
  robotModel::VectorAct desJointPositions, desJointVelocities, desJointTorques;
  virtualModelController_->packDesiredJointSetpoints(desJointModes, desJointPositions, desJointVelocities, desJointTorques);

  LegBase::JointControlModes desiredJointControlModes;
  iLeg = 0;
  for (auto leg : *legs_) {
    if (leg->isAndShouldBeGrounded()) {
      desiredJointControlModes.setConstant(robotModel::AM_Torque);
    } else {
      desiredJointControlModes.setConstant(robotModel::AM_Position);
    }
    leg->setDesiredJointControlModes(desiredJointControlModes);
    leg->setDesiredJointTorques(desJointTorques.block<3,1>(iLeg*3,0));
    iLeg++;
  }

}

} /* namespace loco */

