/*****************************************************************************************
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
 * GaitSwitcherDynamicGaitDefault.hpp
 *
 *  Created on: Aug 22, 2014
 *      Author: gech
 */

#ifndef LOCO_GaitSwitcherDynamicGaitDefault_HPP_
#define LOCO_GaitSwitcherDynamicGaitDefault_HPP_

#include "loco/common/ParameterSet.hpp"
#include "loco/gait_switcher/GaitSwitcherBase.hpp"
#include "loco/gait_switcher/GaitTransition.hpp"
#include "loco/locomotion_controller/LocomotionControllerDynamicGaitDefault.hpp"

#include <memory>
#include <boost/ptr_container/ptr_vector.hpp>

#include "starlethModel/RobotModel.hpp"

namespace loco {

class GaitSwitcherDynamicGaitDefault: public GaitSwitcherBase {
 public:
  GaitSwitcherDynamicGaitDefault(robotModel::RobotModel* robotModel,
                                 robotTerrain::TerrainBase* terrain,
                                 double dt);
  virtual ~GaitSwitcherDynamicGaitDefault();

  virtual bool initialize(double dt);
  virtual bool advance(double dt);

  void setIsRealRobot(bool isRealRobot);

  void setPathToConfigFile(const std::string& pathToConfigFile);
  void setPathToParameterFiles(const std::string& pathToParameterFiles);

  /*! Activates or deactivates the automatic transition depending on the traveling speed
   * @param isActive
   */
  void setAutoTransition(bool isActive);
  bool isTransiting();

  /*! Transit to another gait given by the name of the parameter file
   * @param name  filename of the parameter set
   * @return  true if the gait transition is valid
   */
  bool transitToGait(const std::string& name);
  bool loadParameterSet(int parameterSetIdx);

  bool interpolateParameters(double t);

  LocomotionControllerDynamicGaitDefault* getLocomotionController();

 private:
  bool getLocomotionControllerByName(const std::string& name, LocomotionControllerDynamicGaitDefault* loco);
  bool updateTransition(double simulatedTime);
 private:
  robotModel::RobotModel* robotModel_;
  robotTerrain::TerrainBase* terrain_;
  double time_step_;
  //! If true the parameter files for the real robot are loaded
  bool isRealRobot_;

  //! true if transition is ongoing
  bool isTransiting_;

  //! if true the transit is initialized at the correct time
  bool initTransit_;

  double startTimeOfTransition_;

  double endTimeOfTransition_;

  //! Current gait transition map
  GaitTransition* currentGaitTransition;


  //! Path to the configuration file
  std::string pathToConfigFile_;

  //! path to the folder that contains the parameter files
  std::string pathToParameterFiles_;

  //! if true the transition is triggered automatically
  bool isAutoTransitionOn_;

  double time_;

  double interpolationParameter_;

  //! XML Configuration file
  ParameterSet config_;

  //! Container of admissible gait transitions
  boost::ptr_vector<GaitTransition> gaitTransitions_;

  std::shared_ptr<LocomotionControllerDynamicGaitDefault> locomotionController_;
  boost::ptr_vector<LocomotionControllerDynamicGaitDefault> locomotionControllers_;


};

} /* namespace loco */

#endif /* GaitSwitcherDynamicGaitDefault_HPP_ */
