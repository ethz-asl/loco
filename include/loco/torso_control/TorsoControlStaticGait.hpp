/*
 * TorsoControlStaticGait.hpp
 *
 *  Created on: Oct 9, 2014
 *      Author: dario
 */

#ifndef LOCO_TORSOCONTROLSTATICGAIT_HPP_
#define LOCO_TORSOCONTROLSTATICGAIT_HPP_


#include "loco/torso_control/TorsoControlDynamicGaitFreePlane.hpp"
#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlStaticGait.hpp"

namespace loco {

class TorsoControlStaticGait: public TorsoControlDynamicGaitFreePlane {
 public:
  TorsoControlStaticGait(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain);
  virtual ~TorsoControlStaticGait();

  virtual bool initialize(double dt);

 protected:
//  CoMOverSupportPolygonControlStaticGait* comControl_;

};

} /* namespace loco */


#endif /* LOCO_TORSOCONTROLSTATICGAIT_HPP_ */
