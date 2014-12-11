/*******************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Christian Gehring, Péter Fankhauser, C. Dario Bellicoso, Stelian Coros
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
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
#include "loco/contact_force_distribution/ContactForceDistribution.hpp"

#include "loco/foot_placement_strategy/FootPlacementStrategyInvertedPendulum.hpp"

#include "GaitPatternAPSPreview.hpp"
#include "GaitPatternFlightPhasesPreview.hpp"

#include "AppGUI/TaskVisualizer.h"

namespace loco {

class VisualizerSC: public VisualizerBase, public TaskVisualizer {
 public:
  typedef  Eigen::Matrix<double, 12,1> VectorQj;
 public:
  VisualizerSC(int* drawCharacter = nullptr);
  virtual ~VisualizerSC();

  virtual void addParameters();

  void drawSupportPolygon(loco::LegGroup* legs, double lineWidth = 0.5);
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
  void drawTrajectoryCatMullRomPosition(TrajectoryPosition &trajectory, double dt = 0.1, double lineWidth = 0.5);
  void drawTrajectoryLinearPosition(TrajectoryPosition &trajectory, double dt = 0.1, double lineWidth = 0.5);
  void drawTrajectoryLinearPositionKnots(TrajectoryPosition &trajectory, double lineWidth = 0.5);

  virtual void drawdrawHistoryOfPredictedFootHolds(loco::FootPlacementStrategyInvertedPendulum* strategy);


  virtual void drawFrictionPyramidOfContactForceDistribution(loco::LegGroup* legs, loco::ContactForceDistribution* contactForceDistribution, double heightOfFrictionPyramid=0.15);

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
  int* drawCharacter_;
  bool isSimulationRunning_;
  double desiredFrameRate_;
  loco::TrajectoryPosition footTrajectories_[4];
  loco::TrajectoryPosition desiredFootTrajectories_[4];
  loco::TrajectoryPosition predictedFootHoldTrajectories_[4];
  loco::TrajectoryPosition predictedDefaultFootHoldTrajectories_[4];
  loco::TrajectoryPosition predictedFootHoldInvertedPendulumTrajectories_[4];
  loco::TrajectoryPosition baseTrajectory_;
};

} /* namespace loco */

#endif /* LOCO_VISUALIZERSC_HPP_ */
