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
  virtual bool advance(double dt);

  virtual bool loadParameters(const TiXmlHandle& handle);

  virtual bool setCoMDelta(double delta);

  virtual void setIsInStandConfiguration(bool isInStandConfiguration);
  virtual bool getIsInStandConfiguration() const;

 protected:
//  CoMOverSupportPolygonControlStaticGait* comControl_;

  bool isInStandConfiguration_;

};

} /* namespace loco */


#endif /* LOCO_TORSOCONTROLSTATICGAIT_HPP_ */
