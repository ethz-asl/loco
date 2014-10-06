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

#include "RobotModel.hpp"

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
