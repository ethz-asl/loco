/*
 * CoMOverSupportPolygonStaticGait.hpp
 *
 *  Created on: Oct 7, 2014
 *      Author: dario
 */

#ifndef LOCO_COMOVERSUPPORTPOLYGONCONTROLSTATICGAIT_HPP_
#define LOCO_COMOVERSUPPORTPOLYGONCONTROLSTATICGAIT_HPP_

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlDynamicGait.hpp"

namespace loco {

class CoMOverSupportPolygonControlStaticGait: public CoMOverSupportPolygonControlDynamicGait {
 public:
  CoMOverSupportPolygonControlStaticGait(LegGroup* legs);
  virtual ~CoMOverSupportPolygonControlStaticGait();

  virtual void advance(double dt);
  virtual const Position& getDesiredWorldToCoMPositionInWorldFrame() const;

 protected:
  std::vector<int> swingOrder_;
  Position positionWorldToHorizontalDesiredBaseInWorldFrame_;
  Eigen::Matrix<double,3,4> getNextStanceConfig(Eigen::Matrix<double,3,4> currentStanceConfig, int steppingFoot);
  int getNextSwingFoot(int currentSwingFoot);
  Position lineIntersect(Eigen::Matrix<double,3,2> l1, Eigen::Matrix<double,3,2> l2);
};

} /* namespace loco */

#endif /* LOCO_COMOVERSUPPORTPOLYGONCONTROLSTATICGAIT_HPP_ */
