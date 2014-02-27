/*!
* @file     BaseControlDynamicGait.hpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#ifndef LOCO_BASECONTROLDYNAMICGAIT_HPP_
#define LOCO_BASECONTROLDYNAMICGAIT_HPP_

#include "loco/torso_control/TorsoControlBase.hpp"
#include "loco/torso_control/CoMOverSupportPolygonControl.hpp"

#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"

#include "kindr/rotations/RotationEigen.hpp"

#include "TerrainBase.hpp"
#include "PeriodicRBF1DC1.hpp"

namespace loco {

class TorsoControlDynamicGait: public TorsoControlBase {
 public:
  typedef kindr::rotations::eigen_impl::AngleAxisPD AngleAxis;
 public:
  TorsoControlDynamicGait(LegGroup* legs, TorsoBase* torso, robotTerrain::TerrainBase* terrain);
  virtual ~TorsoControlDynamicGait();

  virtual void advance(double dt);
 private:
  LegGroup* legs_;
  TorsoBase* torso_;
  robotTerrain::TerrainBase* terrain_;
  CoMOverSupportPolygonControl comControl_;

  double headingDistanceFromForeToHindInBasFrame_;
  rbf::PeriodicRBF1DC1 desiredTorsoForeHeightAboveGroundInWorldFrame_;
  rbf::PeriodicRBF1DC1 desiredTorsoHindHeightAboveGroundInWorldFrame_;

};

} /* namespace loco */

#endif /* LOCO_BASECONTROLDYNAMICGAIT_HPP_ */
