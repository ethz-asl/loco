/*
 * CoMOverSupportPolygonStaticGait.hpp
 *
 *  Created on: Oct 7, 2014
 *      Author: dario
 */

#ifndef LOCO_COMOVERSUPPORTPOLYGONCONTROLSTATICGAIT_HPP_
#define LOCO_COMOVERSUPPORTPOLYGONCONTROLSTATICGAIT_HPP_

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlBase.hpp"

namespace loco {

class CoMOverSupportPolygonControlStaticGait: public CoMOverSupportPolygonControlBase {
 public:

  typedef Eigen::Matrix<double,3,4> FeetConfiguration;
  typedef Eigen::Matrix<double,2,2> Line;

  CoMOverSupportPolygonControlStaticGait(LegGroup* legs);
  virtual ~CoMOverSupportPolygonControlStaticGait();

  virtual void advance(double dt);
  virtual bool initialize();

  virtual const Position& getPositionWorldToDesiredCoMInWorldFrame() const;

 protected:

  //! The swing sequence based on the gait plan
  std::vector<int> swingOrder_;

  //! Distance between the safe triangle and the support triangle segments
  double delta_;

  //! 3x4 matrix that stacks (in columns) the starting feet positions
  FeetConfiguration homePos_;

  //! The desired CoM position in world frame
  Position positionWorldToDesiredCoMInWorldFrame_;

  //! Get the next stance feet positions based on the gait planner
  Eigen::Matrix<double,3,4> getNextStanceConfig(Eigen::Matrix<double,3,4> currentStanceConfig, int steppingFoot);

  //! Get the next swing foot based on the gait sequence
  int getNextSwingFoot(int currentSwingFoot);

  //! Find the intersection (if it exists) between two lines
  bool lineIntersect(const Line& l1, const Line& l2, Eigen::Vector2d& intersection);

  //! Get the index of the current swing leg
  int getIndexOfSwingLeg();
};

} /* namespace loco */

#endif /* LOCO_COMOVERSUPPORTPOLYGONCONTROLSTATICGAIT_HPP_ */
