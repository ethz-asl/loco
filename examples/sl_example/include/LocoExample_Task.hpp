/*
 * LocoExample_Task.hpp
 *
 *  Created on: Feb 27, 2014
 *      Author: gech
 */

#ifndef LOCOEXAMPLE_TASK_HPP_
#define LOCOEXAMPLE_TASK_HPP_

#include "TaskRobotBase.hpp"
#include "loco/locomotion_controller/LocomotionControllerDynamicGait.hpp"
#include <memory>

namespace robotTask {

class LocoExample: public robotTask::TaskRobotBase {
 public:
  LocoExample(robotModel::RobotModel* robotModel);
  virtual ~LocoExample();

  /*! Adds the task.
   * @return  true if successful
   */
  virtual bool add();

  /*! Initializes the task.
   * @return  true if successful
   */
  virtual bool init();

  /*! Runs the task.
   * @return  true if successful
   */
  virtual bool run();

  /*! Changes the parameters of the task.
   * Loads a menu where the user can change parameters of this task.
   * @return  true if successful
   */
  virtual bool change();
 private:
  loco::LocomotionControllerDynamicGait* locomotionController_;
  std::shared_ptr<loco::LegGroup> legs_;
};

} /* namespace robotTask */

#endif /* LOCOEXAMPLE_TASK_HPP_ */
