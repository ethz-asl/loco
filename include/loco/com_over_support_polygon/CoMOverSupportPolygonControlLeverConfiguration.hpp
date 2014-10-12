/*
 * CoMOverSupportPolygonControlLeverConfiguration.hpp
 *
 *  Created on: Oct 7, 2014
 *      Author: dario
 */

#ifndef LOCO_COMOVERSUPPORTPOLYGONCONTROLLEVERCONFIGURATION_HPP_
#define LOCO_COMOVERSUPPORTPOLYGONCONTROLLEVERCONFIGURATION_HPP_

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlBase.hpp"

namespace loco {

class CoMOverSupportPolygonControlLeverConfiguration: public CoMOverSupportPolygonControlBase {
 public:
  CoMOverSupportPolygonControlLeverConfiguration(LegGroup* legs, TorsoBase* torso, TerrainModelBase* terrainModel);
  virtual ~CoMOverSupportPolygonControlLeverConfiguration();

  virtual bool setToInterpolated(const CoMOverSupportPolygonControlBase& supportPolygon1, const CoMOverSupportPolygonControlBase& supportPolygon2, double t);

  virtual void advance(double dt);

 protected:
  TorsoBase* torso_;
  TerrainModelBase* terrainModel_;

  Position positionCenterToForeHindSupportFeetInControlFrame_[2];
  Position positionWorldToCenterInWorldFrame_;

};

} /* namespace loco */


#endif /* LOCO_COMOVERSUPPORTPOLYGONCONTROLLEVERCONFIGURATION_HPP_ */
