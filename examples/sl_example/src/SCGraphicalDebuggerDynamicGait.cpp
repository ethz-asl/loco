/*
 * GraphicalDebugger.cpp
 *
 *  Created on: Mar 6, 2014
 *      Author: gech
 */

#include "SCGraphicalDebuggerDynamicGait.hpp"
#include "Globals.h"
#include <AppGUI/TCLTKParameters.h>


namespace loco {

SCGraphicalDebuggerDynamicGait::SCGraphicalDebuggerDynamicGait():
    drawSupportPolygon_(1),
    drawDesiredPose_(1),
    drawMeasuredPose_(0),
    drawTorsoController_(0),
    drawGaitPatternAPS_(1),
    drawContactForces_(0),
    drawDesiredVirtualForces_(1),
    drawDistributedVirtualForces_(1),
    gaitPatternWindow_(nullptr)
{


  ADD_GUI_VISUALIZATION_OPTION_BOOL(&drawMeasuredPose_, "Draw measured pose");
  ADD_GUI_VISUALIZATION_OPTION_BOOL(&drawDesiredPose_, "Draw desired pose");
  ADD_GUI_VISUALIZATION_OPTION_BOOL(&drawSupportPolygon_, "Draw support polygon");
  ADD_GUI_VISUALIZATION_OPTION_BOOL(&drawTorsoController_, "Draw torso controller");
  ADD_GUI_VISUALIZATION_OPTION_BOOL(&drawGaitPatternAPS_, "Draw gait pattern");
  ADD_GUI_VISUALIZATION_OPTION_BOOL(&drawContactForces_, "Draw contact forces");
  ADD_GUI_VISUALIZATION_OPTION_BOOL(&drawDesiredVirtualForces_, "Draw desired virtual forces (red)");
  ADD_GUI_VISUALIZATION_OPTION_BOOL(&drawDistributedVirtualForces_, "Draw distributed virtual forces (blue)");
  gaitPatternWindow_ = new GaitPatternAPSPreview(0, 0, 450, 150);

}

SCGraphicalDebuggerDynamicGait::~SCGraphicalDebuggerDynamicGait() {
  delete gaitPatternWindow_;
}

void SCGraphicalDebuggerDynamicGait::draw(bool shadowMode, Character* character, AbstractRBEngine* world, robotTask::LocoExample* locoTask)
{
  loco::LocomotionControllerDynamicGait* locomotionController = locoTask->getLocomotionController();

  if (shadowMode == false){
    if (drawMeasuredPose_) {
      drawMeasuredPose(character, world, locomotionController->getTorso(), locomotionController->getLegs());
    }

    if (drawDesiredPose_) {
      drawDesiredPose(character, world, locomotionController->getTorso(), locomotionController->getLegs());
    }

    if (drawTorsoController_) {
      loco::Position errorVector = locoTask->torsoController_->getCoMControl()->getPositionErrorVectorInWorldFrame();
      loco::Position posTarget = locoTask->torsoController_->getCoMControl()->getDefaultTarget();
      GLUtils::drawArrow(Vector3d(errorVector.x(), errorVector.y(), errorVector.z()), Point3d(posTarget.x(), posTarget.y(), posTarget.z()));
    }

    if (drawSupportPolygon_) {
      drawSupportPolygon(locomotionController->getLegs());
    }

    if (drawGaitPatternAPS_) {
      drawGaitPatternAPS(locoTask->gaitPatternAPS_.get(), locomotionController->getTorso()->getStridePhase());
    }

    if (drawContactForces_) {
      drawContactForces(world);
    }

    if (drawDesiredVirtualForces_) {
      drawDesiredVirtualForces(locomotionController->getTorso(), locomotionController->getLegs(), locoTask->virtualModelController_.get());
    }

    if (drawDistributedVirtualForces_) {
      drawDistributedVirtualForces(locomotionController->getTorso(), locomotionController->getLegs(), locoTask->virtualModelController_.get());
    }

  }
}

void SCGraphicalDebuggerDynamicGait::drawGaitPatternAPS(loco::GaitPatternAPS* gaitPattern, double stridePhase) {

    if (gaitPatternWindow_ != nullptr) {
      gaitPatternWindow_->gp = gaitPattern;
      gaitPatternWindow_->cursorPosition = stridePhase;
      gaitPatternWindow_->draw();
    }
}

void SCGraphicalDebuggerDynamicGait::drawContactForces(AbstractRBEngine* world) {
  std::vector<ContactForce>* cfs = world->getContactForces();
  GLUtils::glLColor(0, 0, 0);
  for (uint i=0; i<cfs->size();i++) {
    GLUtils::drawArrow(cfs->at(i).f/-300.0, cfs->at(i).cp, 0.005);
  }
}

void SCGraphicalDebuggerDynamicGait::drawDesiredPose(Character* character, AbstractRBEngine* world, loco::TorsoBase* torso, loco::LegGroup* legs) {

  loco::RotationQuaternion  orientationWorldToBaseInWorldFrame;
  loco::Position positionWorldToBaseInWorldFrame;
  orientationWorldToBaseInWorldFrame = torso->getDesiredState().getWorldToBaseOrientationInWorldFrame();
  positionWorldToBaseInWorldFrame = torso->getDesiredState().getWorldToBasePositionInWorldFrame();
  robotModel::VectorQj desJointPositions;
  int iLeg =0;
  for (auto leg : *legs) {
    desJointPositions.block<3,1>(iLeg*3,0) = leg->getDesiredJointPositions();
    iLeg++;
  }
  drawPose(character, world, positionWorldToBaseInWorldFrame, orientationWorldToBaseInWorldFrame, desJointPositions, SHOW_ABSTRACT_VIEW_DESIRED);
}

void SCGraphicalDebuggerDynamicGait::drawMeasuredPose(Character* character, AbstractRBEngine* world,  loco::TorsoBase* torso, loco::LegGroup* legs) {

  loco::RotationQuaternion  orientationWorldToBaseInWorldFrame;
  loco::Position positionWorldToBaseInWorldFrame;
  orientationWorldToBaseInWorldFrame = torso->getMeasuredState().getWorldToBaseOrientationInWorldFrame();
  positionWorldToBaseInWorldFrame = torso->getMeasuredState().getWorldToBasePositionInWorldFrame();
  robotModel::VectorQj desJointPositions;
  int iLeg =0;
  for (auto leg : *legs) {
    desJointPositions.block<3,1>(iLeg*3,0) = leg->getMeasuredJointPositions();
    iLeg++;
  }

  drawPose(character, world, positionWorldToBaseInWorldFrame, orientationWorldToBaseInWorldFrame, desJointPositions, SHOW_ABSTRACT_VIEW);
}

void SCGraphicalDebuggerDynamicGait::drawPose(Character* character, AbstractRBEngine* world, const loco::Position& positionWorldToBaseInWorldFrame, const loco::RotationQuaternion& orientationWorldToBaseInWorldFrame,  const robotModel::VectorQj& desJointPositions, int drawFlags) {

  ReducedCharacterState desiredPose(character->getStateDimension());
  Quaternion quat(orientationWorldToBaseInWorldFrame.w(), orientationWorldToBaseInWorldFrame.x(), orientationWorldToBaseInWorldFrame.y(), orientationWorldToBaseInWorldFrame.z());
  //quat = Quaternion(0,0,0,1);
  desiredPose.setOrientation(quat);
  //desiredPose.setPosition(Point3d(0.2,0.2,0.2));


  const robotModel::VectorQj desJointVelocities = robotModel::VectorQj::Zero();
  setCharacterJointState(desiredPose, desJointPositions,desJointVelocities);
  desiredPose.setPosition(Point3d(positionWorldToBaseInWorldFrame.x(),positionWorldToBaseInWorldFrame.y(),positionWorldToBaseInWorldFrame.z()));
  drawPose(character, world, &desiredPose, drawFlags);
}

void SCGraphicalDebuggerDynamicGait::drawDesiredVirtualForces(loco::TorsoBase* torso, loco::LegGroup* legs, loco::VirtualModelController* virtualModelController) {
  const Force forceInBaseFrame = virtualModelController->getDesiredVirtualForceInBaseFrame();
  const Torque torqueInBaseFrame  = virtualModelController->getDesiredVirtualTorqueInBaseFrame();
  glPushMatrix();
  glColor3d(1.0f, 0.0f, 0.0f);
  drawForceAndTorqueInBaseFrame(forceInBaseFrame, torqueInBaseFrame, torso, legs);
  glPopMatrix();
}


void SCGraphicalDebuggerDynamicGait::drawDistributedVirtualForces(loco::TorsoBase* torso, loco::LegGroup* legs, loco::VirtualModelController* virtualModelController) {
  Force netForceInBaseFrame;
  Torque netTorqueInBaseFrame;

  virtualModelController->getDistributedVirtualForceAndTorqueInBaseFrame(netForceInBaseFrame, netTorqueInBaseFrame);

  glPushMatrix();
  glColor3d(0.0f, 0.0f, 1.0f);
  drawForceAndTorqueInBaseFrame(netForceInBaseFrame, netTorqueInBaseFrame, torso, legs);
  glPopMatrix();
}

void SCGraphicalDebuggerDynamicGait::drawForceAndTorqueInBaseFrame(const Force& forceInBaseFrame, const Torque& torqueInBaseFrame, loco::TorsoBase* torso, loco::LegGroup* legs) {

  double forceScale = 1.0/100.0;

  const Force virtualForceInBaseFrame = forceInBaseFrame;
  const Torque virtualTorqueInBaseFrame = torqueInBaseFrame;
  const Position positionWorldToBaseInWorldFrame = torso->getMeasuredState().getWorldToBasePositionInWorldFrame();

  const Force virtualForceInWorldFrame = forceScale*Force(torso->getMeasuredState().getWorldToBaseOrientationInWorldFrame().inverseRotate(virtualForceInBaseFrame.toImplementation()));

  // force

  GLUtils::drawArrow(Vector3d(virtualForceInWorldFrame.x(), virtualForceInWorldFrame.y(), virtualForceInWorldFrame.z()), Point3d(positionWorldToBaseInWorldFrame.x(), positionWorldToBaseInWorldFrame.y(), positionWorldToBaseInWorldFrame.z()), 0.01, false);

  double lengthHipToHip = legs->getLeftForeLeg()->getBaseToHipPositionInBaseFrame().x()-legs->getLeftHindLeg()->getBaseToHipPositionInBaseFrame().x();
  const Force forceYawInBaseFrame(0.0, lengthHipToHip*virtualTorqueInBaseFrame.z(), 0.0);
  const Force forceYawInWorldFrame = forceScale*Force(torso->getMeasuredState().getWorldToBaseOrientationInWorldFrame().inverseRotate(forceYawInBaseFrame.toImplementation()));

  const Force forcePitchInBaseFrame(0.0, 0.0 , lengthHipToHip*virtualTorqueInBaseFrame.y());
  const Force forcePitchInWorldFrame = forceScale*Force(torso->getMeasuredState().getWorldToBaseOrientationInWorldFrame().inverseRotate(forcePitchInBaseFrame.toImplementation()));

  const Force forceRollInWorldBase(0.0, 0.0 , lengthHipToHip*virtualTorqueInBaseFrame.x());
  const Force forceRollInWorldFrame = forceScale*Force(torso->getMeasuredState().getWorldToBaseOrientationInWorldFrame().inverseRotate(forceRollInWorldBase.toImplementation()));

  const Position foreMidHipInWorldFrame =  Position((legs->getLeftForeLeg()->getWorldToHipPositionInWorldFrame()+ legs->getRightForeLeg()->getWorldToHipPositionInWorldFrame()).toImplementation()*0.5);
  const Position hindMidHipInWorldFrame =  Position((legs->getLeftHindLeg()->getWorldToHipPositionInWorldFrame()+ legs->getRightHindLeg()->getWorldToHipPositionInWorldFrame()).toImplementation()*0.5);
  const Position leftMidHipInWorldFrame =  Position( positionWorldToBaseInWorldFrame.x(), ((legs->getLeftHindLeg()->getWorldToHipPositionInWorldFrame()+ legs->getLeftForeLeg()->getWorldToHipPositionInWorldFrame()).toImplementation()).y()*0.5, positionWorldToBaseInWorldFrame.z());
  const Position rightMidHipInWorldFrame = Position( positionWorldToBaseInWorldFrame.x(), ((legs->getRightHindLeg()->getWorldToHipPositionInWorldFrame()+ legs->getRightForeLeg()->getWorldToHipPositionInWorldFrame()).toImplementation()).y()*0.5, positionWorldToBaseInWorldFrame.z());

  // yaw
  GLUtils::drawArrow(Vector3d(forceYawInWorldFrame.x(), forceYawInWorldFrame.y(), forceYawInWorldFrame.z()), Point3d(foreMidHipInWorldFrame.x(), foreMidHipInWorldFrame.y(), foreMidHipInWorldFrame.z()), 0.01, false);
  GLUtils::drawArrow(Vector3d(-forceYawInWorldFrame.x(), -forceYawInWorldFrame.y(), -forceYawInWorldFrame.z()), Point3d(hindMidHipInWorldFrame.x(), hindMidHipInWorldFrame.y(), hindMidHipInWorldFrame.z()), 0.01, false);
  // pitch
  GLUtils::drawArrow(Vector3d(-forcePitchInWorldFrame.x(), -forcePitchInWorldFrame.y(), -forcePitchInWorldFrame.z()), Point3d(foreMidHipInWorldFrame.x(), foreMidHipInWorldFrame.y(), foreMidHipInWorldFrame.z()), 0.01, false);
  GLUtils::drawArrow(Vector3d(forcePitchInWorldFrame.x(), forcePitchInWorldFrame.y(), forcePitchInWorldFrame.z()), Point3d(hindMidHipInWorldFrame.x(), hindMidHipInWorldFrame.y(), hindMidHipInWorldFrame.z()), 0.01, false);
  // roll
  GLUtils::drawArrow(Vector3d(forceRollInWorldFrame.x(), forceRollInWorldFrame.y(), forceRollInWorldFrame.z()), Point3d(leftMidHipInWorldFrame.x(), leftMidHipInWorldFrame.y(), leftMidHipInWorldFrame.z()), 0.01, false);
  GLUtils::drawArrow(Vector3d(-forceRollInWorldFrame.x(), -forceRollInWorldFrame.y(), -forceRollInWorldFrame.z()), Point3d(rightMidHipInWorldFrame.x(), rightMidHipInWorldFrame.y(), rightMidHipInWorldFrame.z()), 0.01, false);

}




void SCGraphicalDebuggerDynamicGait::drawSupportPolygon(loco::LegGroup* legs) {
  if (legs->getLeftForeLeg()->isGrounded() && legs->getLeftHindLeg()->isGrounded()) {
    const loco::Position start = legs->getLeftForeLeg()->getWorldToFootPositionInWorldFrame();
    const loco::Position end = legs->getLeftHindLeg()->getWorldToFootPositionInWorldFrame();
    GLUtils::drawLine(Point3d(start.x(), start.y(), start.z()), Point3d(end.x(), end.y(), end.z()));
  }
  if (legs->getLeftForeLeg()->isGrounded() && legs->getRightHindLeg()->isGrounded()) {
    const loco::Position start = legs->getLeftForeLeg()->getWorldToFootPositionInWorldFrame();
    const loco::Position end = legs->getRightHindLeg()->getWorldToFootPositionInWorldFrame();
    GLUtils::drawLine(Point3d(start.x(), start.y(), start.z()), Point3d(end.x(), end.y(), end.z()));
  }
  if (legs->getLeftForeLeg()->isGrounded() && legs->getRightForeLeg()->isGrounded()) {
    const loco::Position start = legs->getLeftForeLeg()->getWorldToFootPositionInWorldFrame();
    const loco::Position end = legs->getRightForeLeg()->getWorldToFootPositionInWorldFrame();
    GLUtils::drawLine(Point3d(start.x(), start.y(), start.z()), Point3d(end.x(), end.y(), end.z()));
  }
  if (legs->getRightForeLeg()->isGrounded() && legs->getRightHindLeg()->isGrounded()) {
    const loco::Position start = legs->getRightForeLeg()->getWorldToFootPositionInWorldFrame();
    const loco::Position end = legs->getRightHindLeg()->getWorldToFootPositionInWorldFrame();
    GLUtils::drawLine(Point3d(start.x(), start.y(), start.z()), Point3d(end.x(), end.y(), end.z()));
  }
  if (legs->getRightForeLeg()->isGrounded() && legs->getLeftHindLeg()->isGrounded()) {
    const loco::Position start = legs->getRightForeLeg()->getWorldToFootPositionInWorldFrame();
    const loco::Position end = legs->getLeftHindLeg()->getWorldToFootPositionInWorldFrame();
    GLUtils::drawLine(Point3d(start.x(), start.y(), start.z()), Point3d(end.x(), end.y(), end.z()));
  }
  if (legs->getRightHindLeg()->isGrounded() && legs->getLeftHindLeg()->isGrounded()) {
    const loco::Position start = legs->getRightHindLeg()->getWorldToFootPositionInWorldFrame();
    const loco::Position end = legs->getLeftHindLeg()->getWorldToFootPositionInWorldFrame();
    GLUtils::drawLine(Point3d(start.x(), start.y(), start.z()), Point3d(end.x(), end.y(), end.z()));
  }
}




void SCGraphicalDebuggerDynamicGait::drawPose(Character* character, AbstractRBEngine* world, ReducedCharacterState* desiredPose, int drawFlags) {
  glEnable(GL_LIGHTING);

  std::vector<double> worldState;
  worldState.clear();
  world->getState(&worldState);
  ReducedCharacterState rs(character->getStateDimension());
  character->populateState(&rs);
  //rs.setPosition(rs.getPosition() + Vector3d(0, 0.0, 0));
  rs.setPosition(desiredPose->getPosition() + Vector3d(0, 0.0, 0));
  rs.setOrientation(desiredPose->getOrientation());
  glColor3d(0.0,0.0,0.0);


  copyOrientation(&rs, desiredPose, character->getJointIndex("rfHipAA"));
  copyOrientation(&rs, desiredPose, character->getJointIndex("lfHipAA"));
  copyOrientation(&rs, desiredPose, character->getJointIndex("rrHipAA"));
  copyOrientation(&rs, desiredPose, character->getJointIndex("lrHipAA"));

  copyOrientation(&rs, desiredPose, character->getJointIndex("rfHipFE"));
  copyOrientation(&rs, desiredPose, character->getJointIndex("lfHipFE"));
  copyOrientation(&rs, desiredPose, character->getJointIndex("rrHipFE"));
  copyOrientation(&rs, desiredPose, character->getJointIndex("lrHipFE"));

  copyOrientation(&rs, desiredPose, character->getJointIndex("rfKneeFE"));
  copyOrientation(&rs, desiredPose, character->getJointIndex("lfKneeFE"));
  copyOrientation(&rs, desiredPose, character->getJointIndex("rrKneeFE"));
  copyOrientation(&rs, desiredPose, character->getJointIndex("lrKneeFE"));

  character->setState(&rs);
//  world->drawRBs(SHOW_ABSTRACT_VIEW_SKELETON);
//  world->drawRBs(SHOW_ABSTRACT_VIEW);
  world->drawRBs(drawFlags);
  world->setState(&worldState);

  glDisable(GL_LIGHTING);
}

void SCGraphicalDebuggerDynamicGait::setCharacterJointState(ReducedCharacterState& newState, const robotModel::VectorQj& Qj, const robotModel::VectorQj& dQj) {
  const int jointMapping_[12] = {0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11};

  Vector3d localRotAxis;
  Quaternion tmpRotation;
  for (int i=0;i<Qj.size();i++){
    int jointIndex = jointMapping_[i];
  //      logPrint("joint %d (their) maps to %d(mine): (%s)\n", i, jointIndex, starlETH->getJoint(jointIndex)->name);
    getLocalCoordsRotationAxisForJoint(jointIndex, localRotAxis);
  //      tprintf("local rot axis %d %f %f %f \n",i, localRotAxis.x, localRotAxis.y, localRotAxis.z);
    newState.setJointRelativeAngVelocity(localRotAxis * dQj(i), jointIndex);
    tmpRotation.setToRotationQuaternion(Qj(i), localRotAxis);
    newState.setJointRelativeOrientation(tmpRotation, jointIndex);
  }
}

void  SCGraphicalDebuggerDynamicGait::getLocalCoordsRotationAxisForJoint(int jointIndex, Vector3d& rotationAxis){
  //assume all legs have the same rotation axis...
  const Vector3d legAAAxis = Vector3d(1, 0, 0);
  const Vector3d legFEAxis = Vector3d(0, 1, 0);


  if (isHipAA(jointIndex))
    rotationAxis = legAAAxis;
  else if (isHipFE(jointIndex))
    rotationAxis = legFEAxis;
  else if (isKneeFE(jointIndex))
    rotationAxis = legFEAxis;
  else
    throwError("remove torque component: the joint seems not to be part of any leg!!!\n");
}

bool SCGraphicalDebuggerDynamicGait::isHipAA(int jIndex){
  return jIndex == 0 || jIndex ==  1 || jIndex ==  2 || jIndex == 3;
}

bool SCGraphicalDebuggerDynamicGait::isHipFE(int jIndex){
  return jIndex == 4 || jIndex == 5 || jIndex == 6 || jIndex == 7;
}

bool SCGraphicalDebuggerDynamicGait::isKneeFE(int jIndex){
  return jIndex == 8 || jIndex == 9 || jIndex == 10 || jIndex == 11;
}

} /* namespace loco */
