/*
 * GraphicalDebugger.hpp
 *
 *  Created on: Mar 6, 2014
 *      Author: gech
 */

#ifndef LOCO_GRAPHICALDEBUGGER_HPP_
#define LOCO_GRAPHICALDEBUGGER_HPP_

#include "GLUtils/GLUtils.h"
#include "ControlStarlETH/Character.h"
#include "Physics/AbstractRBEngine.h"
#include "loco/locomotion_controller/LocomotionControllerBase.hpp"
#include "LocoExample_Task.hpp"
#include "GaitPatternAPSPreview.hpp"

namespace loco {

class SCGraphicalDebuggerDynamicGait {
 public:
  SCGraphicalDebuggerDynamicGait();
  virtual ~SCGraphicalDebuggerDynamicGait();
  void draw(bool shadowMode, Character* character, AbstractRBEngine* world, robotTask::LocoExample* task);
  void drawSupportPolygon(loco::LegGroup* legs);
  void drawPose(Character* character, AbstractRBEngine* world, const loco::Position& positionWorldToBaseInWorldFrame, const loco::RotationQuaternion& orientationWorldToBaseInWorldFrame,  const robotModel::VectorQj& desJointPositions, int drawFlags);
  void drawPose(Character* character, AbstractRBEngine* world, ReducedCharacterState* desiredPose, int drawFlags);

  void drawGaitPatternAPS(loco::GaitPatternAPS* gaitPattern, double stridePhase);
  void drawMeasuredPose(Character* character, AbstractRBEngine* world, loco::TorsoBase* torso, loco::LegGroup* legs);
  void drawDesiredPose(Character* character, AbstractRBEngine* world, loco::TorsoBase* torso, loco::LegGroup* legs);
  void setCharacterJointState(ReducedCharacterState& newState, const robotModel::VectorQj& Qj, const robotModel::VectorQj& dQj);
  void getLocalCoordsRotationAxisForJoint(int jointIndex, Vector3d& rotationAxis);

  void drawContactForces(AbstractRBEngine* world);

  /*! Check if the index of a joint of the character indicates a hip AA joint
   * @param jIndex  joint index
   * @return  true if it is a hip AA joint
   */
  bool isHipAA(int jIndex);

  /*! Check if the index of a joint of the character indicates a hip FE joint
   * @param jIndex  joint index
   * @return  true if it is a hip FE joint
   */
  bool isHipFE(int jIndex);

  /*! Check if the index of a joint of the character indicates a knee FE joint
   * @param jIndex  joint index
   * @return  true if it is a knee FE joint
   */
  bool isKneeFE(int jIndex);

 public:
  int drawSupportPolygon_;
  int drawDesiredPose_;
  int drawMeasuredPose_;
  int drawTorsoController_;
  int drawGaitPatternAPS_;
  int drawContactForces_;

  loco::GaitPatternAPSPreview* gaitPatternWindow_;
};

} /* namespace loco */

#endif /* LOCO_GRAPHICALDEBUGGER_HPP_ */
