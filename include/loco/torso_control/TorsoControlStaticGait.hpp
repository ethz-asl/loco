/*
 * TorsoControlStaticGait.hpp
 *
 *  Created on: Oct 9, 2014
 *      Author: dario
 */

#ifndef LOCO_TORSOCONTROLSTATICGAIT_HPP_
#define LOCO_TORSOCONTROLSTATICGAIT_HPP_


#include "loco/torso_control/TorsoControlGaitContainer.hpp"
#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlStaticGait.hpp"

namespace loco {

class TorsoControlStaticGait: public TorsoControlGaitContainer {
 public:
  TorsoControlStaticGait(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain);
  virtual ~TorsoControlStaticGait();

  virtual bool initialize(double dt);
  virtual bool advance(double dt);

  virtual bool loadParameters(const TiXmlHandle& handle);

  virtual void setIsInStandConfiguration(bool isInStandConfiguration);
  virtual bool getIsInStandConfiguration() const;

  virtual bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t);

 protected:
  bool isInStandConfiguration_;

};

} /* namespace loco */


#endif /* LOCO_TORSOCONTROLSTATICGAIT_HPP_ */
