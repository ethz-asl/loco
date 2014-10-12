/*
 * TorsoControlDynamicGaitFreePlane.hpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Dario Bellicoso
 */

#ifndef LOCO_TORSOCONTROLDYNAMICGAITFREEPLANE_HPP_
#define LOCO_TORSOCONTROLDYNAMICGAITFREEPLANE_HPP_

#include "loco/torso_control/TorsoControlBase.hpp"
#include "../robotBase/robotUtils/filters/lowpass/FirstOrderFilter.hpp"

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlBase.hpp"
#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"
#include "loco/common/TerrainModelBase.hpp"

namespace loco {

  class TorsoControlDynamicGaitFreePlane: public TorsoControlBase {
   public:
    enum AdaptToTerrain {CompleteAdaption, SaturatedLinearAdaption};
    TorsoControlDynamicGaitFreePlane(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain);
    TorsoControlDynamicGaitFreePlane(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain, CoMOverSupportPolygonControlBase* comControl);
    virtual ~TorsoControlDynamicGaitFreePlane();


  virtual bool initialize(double dt);
  virtual bool advance(double dt);

  virtual bool loadParameters(const TiXmlHandle& handle);

  const CoMOverSupportPolygonControlBase& getCoMOverSupportPolygonControl() const;

  virtual bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t);

 protected:
  LegGroup* legs_;
  TorsoBase* torso_;
  loco::TerrainModelBase* terrain_;
  CoMOverSupportPolygonControlBase* comControl_;

  double desiredTorsoCoMHeightAboveGroundInControlFrameOffset_;

  //--- Terrain adaptation parameters
  double maxDesiredPitchRadians_;
  double desiredPitchSlope_;
  double maxDesiredRollRadians_;
  double desiredRollSlope_;
  AdaptToTerrain adaptToTerrain_;
  //---

  //--- Height offset parameters
  //double maxHeightOffset_;
  robotUtils:: FirstOrderFilter* firstOrderFilter_;
  //---

  template <typename T> int sgn(T val);

  void getDesiredBasePitchFromTerrainPitch(const double terrainPitch, double& desiredBasePitch);
  void getDesiredBaseRollFromTerrainRoll(const double terrainRoll, double& desiredBaseRoll);

  RotationQuaternion getOrientationHeadingToDesiredHeadingBasedOnFeetLocations(const Position& positionWorldToDesiredHorizontalBaseInWorldFrame) const;
  RotationQuaternion getOrientationWorldToHeadingOnTerrainSurface(const RotationQuaternion& orientationWorldToHeading) const;
  RotationQuaternion getOrientationWorldToHeadingBasedOnHipLocations() const;
  double heightOffsetFilterAdvance(double dt);

};

} /* namespace loco */


#endif /* LOCO_TORSOCONTROLDYNAMICGAITFREEPLANE_HPP_ */
