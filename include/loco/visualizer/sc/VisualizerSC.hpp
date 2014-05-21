/*
 * VisualizerSC.hpp
 *
 *  Created on: Mar 14, 2014
 *      Author: gech
 */

#ifndef LOCO_VISUALIZERSC_HPP_
#define LOCO_VISUALIZERSC_HPP_

#include "loco/visualizer/VisualizerBase.hpp"

#include "GLUtils/GLUtils.h"
#include "GLUtils/GLUtilsKindr.h"
#include "ControlStarlETH/Character.h"
#include "Physics/AbstractRBEngine.h"

#include "loco/locomotion_controller/LocomotionControllerBase.hpp"
#include "loco/motion_control/VirtualModelController.hpp"
#include "GaitPatternAPSPreview.hpp"
#include "GaitPatternFlightPhasesPreview.hpp"

#include "AppGUI/TaskVisualizer.h"

namespace loco {

class VisualizerSC: public VisualizerBase, public TaskVisualizer {
 public:
  typedef  Eigen::Matrix<double, 12,1> VectorQj;
 public:
  VisualizerSC();
  virtual ~VisualizerSC();

  virtual void addParameters();

  void drawSupportPolygon(loco::LegGroup* legs);
  void drawPose(Character* character, AbstractRBEngine* world, const loco::Position& positionWorldToBaseInWorldFrame, const loco::RotationQuaternion& orientationWorldToBaseInWorldFrame,  const VectorQj& desJointPositions, int drawFlags);
  void drawPose(Character* character, AbstractRBEngine* world, ReducedCharacterState* desiredPose, int drawFlags);

  void drawGaitPatternAPS(loco::GaitPatternAPS* gaitPattern, double stridePhase);
  void drawGaitPatternFlightPhases(loco::GaitPatternFlightPhases* gaitPattern);
  void drawMeasuredPose(Character* character, AbstractRBEngine* world, loco::TorsoBase* torso, loco::LegGroup* legs);
  void drawDesiredPose(Character* character, AbstractRBEngine* world, loco::TorsoBase* torso, loco::LegGroup* legs);
  void setCharacterJointState(ReducedCharacterState& newState, const VectorQj& Qj, const VectorQj& dQj);
  void getLocalCoordsRotationAxisForJoint(int jointIndex, Vector3d& rotationAxis);

  void drawContactForces(AbstractRBEngine* world);
  void drawDesiredVirtualForces(loco::TorsoBase* torso, loco::LegGroup* legs, loco::VirtualModelController* virtualModelController);
  void drawForceAndTorqueInBaseFrame(const Force& forceInBaseFrame, const Torque& torqueInBaseFrame, loco::TorsoBase* torso, loco::LegGroup* legs);
  void drawDistributedVirtualForces(loco::TorsoBase* torso, loco::LegGroup* legs, loco::VirtualModelController* virtualModelController);


  void drawHistoryOfFootPositions(loco::LegGroup* legs);
  void drawHistoryOfDesiredFootPositions(loco::LegGroup* legs);
  void drawHistoryOfBasePosition(loco::TorsoBase* torso);
  void drawTrajectoryCatMullRomPosition(TrajectoryPosition &c, double dt = 0.1);

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



  loco::GaitPatternAPSPreview* gaitPatternWindow_;
  loco::GaitPatternFlightPhasesPreview* gaitPatternFlightPhasesWindow_;

 protected:
  double desiredFrameRate_;
  loco::TrajectoryPosition footTrajectories_[4];
  loco::TrajectoryPosition desiredFootTrajectories_[4];
  loco::TrajectoryPosition baseTrajectory_;
};

} /* namespace loco */

#endif /* LOCO_VISUALIZERSC_HPP_ */
