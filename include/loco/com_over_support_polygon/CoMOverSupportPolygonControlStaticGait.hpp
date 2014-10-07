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

  virtual const Position& getDesiredWorldToCoMPositionInWorldFrame() const;

 protected:

  std::vector<int> swingOrder_;

  FeetConfiguration homePos_;

  Position positionWorldToHorizontalDesiredBaseInWorldFrame_;
  Eigen::Matrix<double,3,4> getNextStanceConfig(Eigen::Matrix<double,3,4> currentStanceConfig, int steppingFoot);
  int getNextSwingFoot(int currentSwingFoot);
  Position lineIntersect(Line l1, Line l2);
};

} /* namespace loco */

#endif /* LOCO_COMOVERSUPPORTPOLYGONCONTROLSTATICGAIT_HPP_ */
