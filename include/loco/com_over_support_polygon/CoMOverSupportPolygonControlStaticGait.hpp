/*
 * CoMOverSupportPolygonStaticGait.hpp
 *
 *  Created on: Oct 7, 2014
 *      Author: dario
 */

#ifndef LOCO_COMOVERSUPPORTPOLYGONCONTROLSTATICGAIT_HPP_
#define LOCO_COMOVERSUPPORTPOLYGONCONTROLSTATICGAIT_HPP_

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlBase.hpp"
#include "loco/common/TorsoBase.hpp"

namespace loco {

class CoMOverSupportPolygonControlStaticGait: public CoMOverSupportPolygonControlBase {
 public:

  typedef Eigen::Matrix<double,2,4> FeetConfiguration;
  typedef Eigen::Matrix<double,2,4> SupportTriangle;
  typedef Eigen::Matrix<double,2,2> Line;
  typedef Eigen::Vector2d Pos2d;

  CoMOverSupportPolygonControlStaticGait(LegGroup* legs, TorsoBase* torso);
  virtual ~CoMOverSupportPolygonControlStaticGait();

  virtual void advance(double dt);
  virtual bool initialize();

  virtual const Position& getPositionWorldToDesiredCoMInWorldFrame() const;

 protected:

  TorsoBase* torso_;

  int swingLegIndexNow_;
  int swingLegIndexNext_;
  int swingLegIndexLast_;
  int swingLegIndexOverNext_;
  Eigen::Vector2d comStep_;
  double maxComStep_;
  FeetConfiguration feetConfigurationCurrent_, feetConfigurationNext_, feetConfigurationOnverNext_, homePos_;

  bool swingFootChanged_;

  //! The swing sequence based on the gait plan
  std::vector<int> swingOrder_;

  //! Distance between the safe triangle and the support triangle segments
  double delta_;

  //! The desired CoM position in world frame
  Position positionWorldToDesiredCoMInWorldFrame_;

  //! Get the next stance feet positions based on the gait planner
  Eigen::Matrix<double,2,4> getNextStanceConfig(const FeetConfiguration& currentStanceConfig, int steppingFoot);

  //! Get safe triangle from support triangle
  Eigen::Matrix<double,2,3> getSafeTriangle(const Eigen::Matrix<double,2,3>& supportTriangle);

  //! Get the next swing foot based on the gait sequence
  int getNextSwingFoot(const int currentSwingFoot);

  //! Find the intersection (if it exists) between two lines
  bool lineIntersect(const Line& l1, const Line& l2, Pos2d& intersection);

  //! Get the index of the current swing leg
  int getIndexOfSwingLeg();

  std::vector<int> getDiagonalElements(int swingLeg);

  void updateSwingLegsIndexes();

};

} /* namespace loco */

#endif /* LOCO_COMOVERSUPPORTPOLYGONCONTROLSTATICGAIT_HPP_ */
