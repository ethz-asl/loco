/*
 * VisualizerSC.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: gech
 */

#include "loco/visualizer/sc/VisualizerSC.hpp"
#include "Globals.h"

namespace loco {

VisualizerSC::VisualizerSC() :
 gaitPatternWindow_(nullptr)
{

  gaitPatternWindow_ = new GaitPatternAPSPreview(0, 0, 450, 150);

}
VisualizerSC::~VisualizerSC() {
  delete gaitPatternWindow_;
}


void VisualizerSC::drawGaitPatternAPS(loco::GaitPatternAPS* gaitPattern, double stridePhase) {

    if (gaitPatternWindow_ != nullptr) {
      gaitPatternWindow_->gp = gaitPattern;
      gaitPatternWindow_->cursorPosition = stridePhase;
      gaitPatternWindow_->draw();
    }
}

void VisualizerSC::drawContactForces(AbstractRBEngine* world) {
  std::vector<ContactForce>* cfs = world->getContactForces();
  GLUtils::glLColor(0, 0, 0);
  for (uint i=0; i<cfs->size();i++) {
    GLUtils::drawArrow(cfs->at(i).f/-300.0, cfs->at(i).cp, 0.005);
  }
}

void VisualizerSC::drawDesiredPose(Character* character, AbstractRBEngine* world, loco::TorsoBase* torso, loco::LegGroup* legs) {

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

void VisualizerSC::drawMeasuredPose(Character* character, AbstractRBEngine* world,  loco::TorsoBase* torso, loco::LegGroup* legs) {

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

  drawPose(character, world, positionWorldToBaseInWorldFrame, orientationWorldToBaseInWorldFrame, desJointPositions, SHOW_ABSTRACT_VIEW_MEASURED);
}

void VisualizerSC::drawPose(Character* character, AbstractRBEngine* world, const loco::Position& positionWorldToBaseInWorldFrame, const loco::RotationQuaternion& orientationWorldToBaseInWorldFrame,  const robotModel::VectorQj& desJointPositions, int drawFlags) {

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

void VisualizerSC::drawDesiredVirtualForces(loco::TorsoBase* torso, loco::LegGroup* legs, loco::VirtualModelController* virtualModelController) {
  const Force forceInBaseFrame = virtualModelController->getDesiredVirtualForceInBaseFrame();
  const Torque torqueInBaseFrame  = virtualModelController->getDesiredVirtualTorqueInBaseFrame();
  glPushMatrix();
  GLUtilsKindr::glLColor(1.0, 0.0, 0.0, 1.0);
  drawForceAndTorqueInBaseFrame(forceInBaseFrame, torqueInBaseFrame, torso, legs);
  glPopMatrix();
}


void VisualizerSC::drawDistributedVirtualForces(loco::TorsoBase* torso, loco::LegGroup* legs, loco::VirtualModelController* virtualModelController) {
  Force netForceInBaseFrame;
  Torque netTorqueInBaseFrame;

  virtualModelController->getDistributedVirtualForceAndTorqueInBaseFrame(netForceInBaseFrame, netTorqueInBaseFrame);

  glPushMatrix();
  GLUtilsKindr::glLColor(0.0, 0.0, 1.0, 1.0);
  drawForceAndTorqueInBaseFrame(netForceInBaseFrame, netTorqueInBaseFrame, torso, legs);
  glPopMatrix();
}

void VisualizerSC::drawForceAndTorqueInBaseFrame(const Force& forceInBaseFrame, const Torque& torqueInBaseFrame, loco::TorsoBase* torso, loco::LegGroup* legs) {

  double forceScale = 1.0/100.0;

  const Force virtualForceInBaseFrame = forceInBaseFrame;
  const Torque virtualTorqueInBaseFrame = torqueInBaseFrame;
  const Position positionWorldToBaseInWorldFrame = torso->getMeasuredState().getWorldToBasePositionInWorldFrame();

  const Force virtualForceInWorldFrame = forceScale*Force(torso->getMeasuredState().getWorldToBaseOrientationInWorldFrame().inverseRotate(virtualForceInBaseFrame.toImplementation()));

  // force

  GLUtilsKindr::drawArrow(GLUtilsKindr::Vector(virtualForceInWorldFrame), positionWorldToBaseInWorldFrame, 0.01, false);

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
  GLUtilsKindr::drawArrow(GLUtilsKindr::Vector(forceYawInWorldFrame), foreMidHipInWorldFrame, 0.01, false);
  GLUtilsKindr::drawArrow(GLUtilsKindr::Vector(-forceYawInWorldFrame), hindMidHipInWorldFrame, 0.01, false);
  // pitch
  GLUtilsKindr::drawArrow(GLUtilsKindr::Vector(-forcePitchInWorldFrame), foreMidHipInWorldFrame, 0.01, false);
  GLUtilsKindr::drawArrow(GLUtilsKindr::Vector(forcePitchInWorldFrame), hindMidHipInWorldFrame, 0.01, false);
  // roll
  GLUtilsKindr::drawArrow(GLUtilsKindr::Vector(forceRollInWorldFrame), leftMidHipInWorldFrame, 0.01, false);
  GLUtilsKindr::drawArrow(GLUtilsKindr::Vector(-forceRollInWorldFrame), rightMidHipInWorldFrame, 0.01, false);

}




void VisualizerSC::drawSupportPolygon(loco::LegGroup* legs) {
  if (legs->getLeftForeLeg()->isGrounded() && legs->getLeftHindLeg()->isGrounded()) {
    const loco::Position start = legs->getLeftForeLeg()->getWorldToFootPositionInWorldFrame();
    const loco::Position end = legs->getLeftHindLeg()->getWorldToFootPositionInWorldFrame();
    GLUtilsKindr::drawLine(start, end);
  }
  if (legs->getLeftForeLeg()->isGrounded() && legs->getRightHindLeg()->isGrounded()) {
    const loco::Position start = legs->getLeftForeLeg()->getWorldToFootPositionInWorldFrame();
    const loco::Position end = legs->getRightHindLeg()->getWorldToFootPositionInWorldFrame();
    GLUtilsKindr::drawLine(start, end);
  }
  if (legs->getLeftForeLeg()->isGrounded() && legs->getRightForeLeg()->isGrounded()) {
    const loco::Position start = legs->getLeftForeLeg()->getWorldToFootPositionInWorldFrame();
    const loco::Position end = legs->getRightForeLeg()->getWorldToFootPositionInWorldFrame();
    GLUtilsKindr::drawLine(start, end);
  }
  if (legs->getRightForeLeg()->isGrounded() && legs->getRightHindLeg()->isGrounded()) {
    const loco::Position start = legs->getRightForeLeg()->getWorldToFootPositionInWorldFrame();
    const loco::Position end = legs->getRightHindLeg()->getWorldToFootPositionInWorldFrame();
    GLUtilsKindr::drawLine(start, end);
  }
  if (legs->getRightForeLeg()->isGrounded() && legs->getLeftHindLeg()->isGrounded()) {
    const loco::Position start = legs->getRightForeLeg()->getWorldToFootPositionInWorldFrame();
    const loco::Position end = legs->getLeftHindLeg()->getWorldToFootPositionInWorldFrame();
    GLUtilsKindr::drawLine(start, end);
  }
  if (legs->getRightHindLeg()->isGrounded() && legs->getLeftHindLeg()->isGrounded()) {
    const loco::Position start = legs->getRightHindLeg()->getWorldToFootPositionInWorldFrame();
    const loco::Position end = legs->getLeftHindLeg()->getWorldToFootPositionInWorldFrame();
    GLUtilsKindr::drawLine(start, end);
  }
}




void VisualizerSC::drawPose(Character* character, AbstractRBEngine* world, ReducedCharacterState* desiredPose, int drawFlags) {
  glEnable(GL_LIGHTING);

  std::vector<double> worldState;
  worldState.clear();
  world->getState(&worldState);
  ReducedCharacterState rs(character->getStateDimension());
  character->populateState(&rs);
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
//  world->drawRBs(drawFlags);

  bool shadowMode = false;
  bool drawMesh = false;
  int flags = (drawMesh)?(SHOW_MESH | SHOW_EYE_BLINK):(SHOW_ABSTRACT_VIEW | drawFlags /*| SHOW_BODY_FRAME | SHOW_ABSTRACT_VIEW*/);

    flags |= SHOW_CHARACTER;
//  flags = SHOW_ABSTRACT_VIEW;
  //if we are drawing shadows, we don't need to enable textures or lighting, since we only want the projection anyway
  glEnable(GL_LIGHTING);
  if (shadowMode == false){
    flags |= SHOW_CPS_IF_NO_MESH;
    flags |= SHOW_COLOURS/* | SHOW_JOINTS | SHOW_BODY_FRAME*/;
    glEnable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_LIGHTING);

  }else
    glDisable(GL_LIGHTING);

  if (world == NULL)
    return;

  character->getAF()->root->draw(flags);
  for (int i=0; i<character->getJointCount(); i++) {
      character->getJoint(i)->getChild()->draw(flags);
  }
//  world->drawRBs(flags);
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);
  glColor3d(0,0,0);






  world->setState(&worldState);

  glDisable(GL_LIGHTING);
}

void VisualizerSC::setCharacterJointState(ReducedCharacterState& newState, const robotModel::VectorQj& Qj, const robotModel::VectorQj& dQj) {
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

void  VisualizerSC::getLocalCoordsRotationAxisForJoint(int jointIndex, Vector3d& rotationAxis){
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

bool VisualizerSC::isHipAA(int jIndex){
  return jIndex == 0 || jIndex ==  1 || jIndex ==  2 || jIndex == 3;
}

bool VisualizerSC::isHipFE(int jIndex){
  return jIndex == 4 || jIndex == 5 || jIndex == 6 || jIndex == 7;
}

bool VisualizerSC::isKneeFE(int jIndex){
  return jIndex == 8 || jIndex == 9 || jIndex == 10 || jIndex == 11;
}

} /* namespace loco */
