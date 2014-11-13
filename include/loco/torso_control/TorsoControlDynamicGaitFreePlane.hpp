/*
 * TorsoControlDynamicGaitFreePlane.hpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Dario Bellicoso
 */

#ifndef LOCO_TORSOCONTROLDYNAMICGAITFREEPLANE_HPP_
#define LOCO_TORSOCONTROLDYNAMICGAITFREEPLANE_HPP_

#include "loco/torso_control/TorsoControlGaitContainer.hpp"
#include "robotUtils/filters/FirstOrderFilter.hpp"

namespace loco {

class TorsoControlDynamicGaitFreePlane: public TorsoControlGaitContainer {

  public:
    enum AdaptToTerrain {CompleteAdaption, SaturatedLinearAdaption};
    TorsoControlDynamicGaitFreePlane(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain);
    virtual ~TorsoControlDynamicGaitFreePlane();


    virtual bool initialize(double dt);
    virtual bool advance(double dt);

    virtual bool loadParameters(const TiXmlHandle& handle);

    virtual bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t);


  protected:
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

};

} /* namespace loco */


#endif /* LOCO_TORSOCONTROLDYNAMICGAITFREEPLANE_HPP_ */
