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
  virtualModelController_->loadParameters();
  contactForceDistribution_->loadParameters();
  return true;
}

void LocomotionControllerDynamicGait::advance(double dt) {

  Eigen::Vector4i contactFlags = robotModel_->contacts().getCA();
  int iLeg = 0;
  for (auto leg : *legs_) {
    if (contactFlags(iLeg) == 1) {
      leg->setIsGrounded(true);
    }
    else {
      leg->setIsGrounded(false);
    }
    iLeg++;
  }
  const Eigen::Vector4d vQuat = robotModel_->est().getActualEstimator()->getQuat();
  const TorsoBase::Pose::Rotation rquatWorldToBase(vQuat(0), vQuat(1), vQuat(2), vQuat(3));
  const TorsoBase::Pose::Position position(rquatWorldToBase.rotate(robotModel_->kin()[robotModel::JT_World2Base_CSw]->getPos()));
  torso_->setMeasuredPoseInWorldFrame(TorsoBase::Pose(position, rquatWorldToBase));
  const TorsoBase::Twist::PositionDiff linearVelocity(rquatWorldToBase.rotate(robotModel_->kin()[robotModel::JT_World2Base_CSw]->getVel()));
  const TorsoBase::Twist::RotationDiff localAngularVelocity(rquatWorldToBase.rotate(robotModel_->kin()(robotModel::JR_World2Base_CSw)->getOmega()));
  torso_->setMeasuredTwistInBaseFrame(TorsoBase::Twist(linearVelocity, localAngularVelocity));

  limbCoordinator_->advance(dt);

  iLeg = 0;
  for (auto leg : *legs_) {
    const double swingPhase = leg->getSwingPhase();
    if ((swingPhase >= 0 && swingPhase <= 0.5) && leg->isInStanceMode()) {
      // possible lift-off
      leg->getStateLiftOff()->setFootPositionInWorldFrame(robotModel_->kin().getJacobianTByLeg_Base2Foot_CSw(iLeg)->getPos());
      leg->getStateLiftOff()->setHipPositionInWorldFrame(robotModel_->kin().getJacobianTByLeg_World2Hip_CSw(iLeg)->getPos());
      iLeg++;
    }
  }
  footPlacementStrategy_->advance(dt);
  iLeg = 0;
  for (auto leg : *legs_) {
    Eigen::Vector3d  pos = footPlacementStrategy_->getFootPositionForSwingLegCSw(iLeg, dt)
                - robotModel_->kin().getJacobianTByLeg_World2Hip_CSw(iLeg)->getPos();
    Eigen::Vector3d positionHipToFootInHipFrame = torso_->getMeasuredPoseInWorldFrame().getRotation().rotate(pos );
    leg->setDesiredJointPositions(robotModel_->kin().getJointPosFromFootPos(positionHipToFootInHipFrame, iLeg));
    iLeg++;
  }
  torsoController_->advance(dt);



  //! Desired base position expressed in inertial frame.
  robotModel::VectorP baseDesiredPosition(0.0, 0.0, 0.475);
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
    if (leg->isAndShoulBeGrounded()) {
      desiredJointControlModes.setConstant(robotModel::AM_Torque);
    } else {
      desiredJointControlModes.setConstant(robotModel::AM_Position);
    }
    leg->setDesiredJointControlModes(desiredJointControlModes);
    leg->setDesiredJointTorques(desJointTorques.block<3,1>(iLeg*3,0));
    iLeg++;
  }
  // inverse kin -> des joint pos for swing leg
  // force control -> des torque for stance leg

}

} /* namespace loco */

