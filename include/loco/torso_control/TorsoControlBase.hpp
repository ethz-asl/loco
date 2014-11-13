/*!
* @file     TorsoControlBase.hpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/

#ifndef LOCO_TORSOCONTROLBASE_HPP_
#define LOCO_TORSOCONTROLBASE_HPP_

#include "tinyxml.h"
#include "robotUtils/function_approximators/polyharmonicSplines/PeriodicRBF1DC1.hpp"
#include "loco/common/TypeDefs.hpp"
#include "kindr/rotations/RotationEigen.hpp"

namespace loco {

class TorsoControlBase {

 public:
  TorsoControlBase();
  virtual ~TorsoControlBase();
  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;
  virtual bool loadParameters(const TiXmlHandle& handle) = 0;

  /*! Computes an interpolated version of the two controllers passed in as parameters.
   *  if t is 0, the current setting is set to controller1, 1 -> controller2, and values in between
   *  correspond to interpolated parameter set.
   * @param torsoController1
   * @param torsoController2
   * @param t interpolation parameter
   * @returns true if successful
   */
  virtual bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t) = 0;

  /*! Set a position offset to the CoM in world frame (used by the mission controller to move the robot while standing)
   * @param[in] positionOffsetInWorldFrame The position offset
   */
  virtual void setDesiredPositionOffsetInWorldFrame(const Position& positionOffsetInWorldFrame) = 0;

  /*! Set an attitude offset to the CoM w.r.t. world frame (used by the mission controller to change the robot attitude while standing)
   * @param[in] orientationOffset The orientation offset
   */
  virtual void setDesiredOrientationOffset(const RotationQuaternion& orientationOffset) = 0;

};

} /* namespace loco */

#endif /* LOCO_TORSOCONTROLBASE_HPP_ */
