/*
 * TorsoControlDynamicGaitFreePlane.hpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Dario Bellicoso
 */

#ifndef LOCO_TORSOCONTROLDYNAMICGAITFREEPLANE_HPP_
#define LOCO_TORSOCONTROLDYNAMICGAITFREEPLANE_HPP_


#include "loco/torso_control/TorsoControlBase.hpp"
#include "loco/torso_control/CoMOverSupportPolygonControl.hpp"

#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"

#include "kindr/rotations/RotationEigen.hpp"

#include "loco/common/TerrainModelBase.hpp"

#include "PeriodicRBF1DC1.hpp"

namespace loco {

  class TorsoControlDynamicGaitFreePlane: public TorsoControlBase {
   public:
    TorsoControlDynamicGaitFreePlane(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain);
    virtual ~TorsoControlDynamicGaitFreePlane();

    virtual bool initialize(double dt);
    virtual bool advance(double dt);

    virtual RotationQuaternion computeHeading(const RotationQuaternion& rquat, const Vector& axis);
    RotationQuaternion decomposeRotation(const RotationQuaternion& q_BA, const Vector& vB);
    CoMOverSupportPolygonControl* getCoMControl();
    const CoMOverSupportPolygonControl& getCoMControl() const;
    virtual bool loadParameters(const TiXmlHandle& handle);

    double getDesiredTorsoForeHeightAboveGroundInWorldFrameOffset() const;
    double getDesiredTorsoHindHeightAboveGroundInWorldFrameOffset() const;

    void setDesiredPositionOffetInWorldFrame(const Position& positionOffsetInWorldFrame);
    void setDesiredOrientationOffset(const RotationQuaternion& orientationOffset);

    /*! Computes an interpolated version of the two controllers passed in as parameters.
     *  if t is 0, the current setting is set to controller1, 1 -> controller2, and values in between
     *  correspond to interpolated parameter set.
     * @param torsoController1
     * @param torsoController2
     * @param t interpolation parameter
     * @returns true if successful
     */
    virtual bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t);

   private:
    LegGroup* legs_;
    TorsoBase* torso_;
    loco::TerrainModelBase* terrain_;
    CoMOverSupportPolygonControl comControl_;

    double headingDistanceFromForeToHindInBaseFrame_;
    double desiredTorsoForeHeightAboveGroundInWorldFrameOffset_;
    double desiredTorsoHindHeightAboveGroundInWorldFrameOffset_;
    rbf::PeriodicRBF1DC1 desiredTorsoForeHeightAboveGroundInWorldFrame_;
    rbf::PeriodicRBF1DC1 desiredTorsoHindHeightAboveGroundInWorldFrame_;
    Position desiredPositionOffetInWorldFrame_;
    RotationQuaternion desiredOrientationOffset_;
   private:
    virtual bool loadParametersHipConfiguration(const TiXmlHandle &hParameterSet);
    virtual bool loadHeightTrajectory(const TiXmlHandle &hTrajectory,  rbf::PeriodicRBF1DC1& trajectory);
    bool interpolateHeightTrajectory(rbf::PeriodicRBF1DC1& interpolatedTrajectory, const rbf::PeriodicRBF1DC1& trajectory1, const rbf::PeriodicRBF1DC1& trajectory2, double t);


  };

} /* namespace loco */


#endif /* LOCO_TORSOCONTROLDYNAMICGAITFREEPLANE_HPP_ */
