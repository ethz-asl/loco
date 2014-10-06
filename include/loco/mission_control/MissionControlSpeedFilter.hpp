/*
 * MissionControlSpeedFilter.hpp
 *
 *  Created on: Mar 7, 2014
 *      Author: gech
 */

#ifndef LOCO_MissionControlSpeedFilter_HPP_
#define LOCO_MissionControlSpeedFilter_HPP_

#include "loco/mission_control/MissionControlBase.hpp"
#include "loco/temp_helpers/FilteredVariable.hpp"

namespace loco {

class MissionControlSpeedFilter: public MissionControlBase {
 public:
  MissionControlSpeedFilter();
  virtual ~MissionControlSpeedFilter();

  virtual bool initialize(double dt);
  virtual bool advance(double dt);
  const Twist& getDesiredBaseTwistInHeadingFrame() const;
  const Twist& getMaximumBaseTwistInHeadingFrame() const;
  void setUnfilteredDesiredBaseTwistInHeadingFrame(const Twist& twist);
  void setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(const Position& position);

  virtual bool loadParameters(const TiXmlHandle& handle);

  bool setToInterpolated(const MissionControlBase& missionController1, const MissionControlBase& missionController2, double t);

  const Position& getDesiredPositionOffsetInWorldFrame() const;
  const Position& getMinimalPositionOffsetInWorldFrame() const;
  const Position& getMaximalPositionOffsetInWorldFrame() const;



  const RotationQuaternion& getDesiredOrientationOffset() const;
  void setUnfilteredDesiredOrientationOffset(const RotationQuaternion& desiredOrientationHeadingToBase);

  const EulerAnglesZyx& getMinimalDesiredOrientationOffset() const;
  const EulerAnglesZyx& getMaximalDesiredOrientationOffset() const;

  friend std::ostream& operator << (std::ostream& out, const MissionControlSpeedFilter& speedFilter);

 protected:
  Twist baseTwistInHeadingFrame_;
  Twist unfilteredBaseTwistInHeadingFrame_;
  Twist maximumBaseTwistInHeadingFrame_;
  Position desiredPositionOffsetInWorldFrame_;
  Position minimalDesiredPositionOffsetInWorldFrame_;
  Position maximalDesiredPositionOffsetInWorldFrame_;
  RotationQuaternion desiredOrientationHeadingToBase_;
  EulerAnglesZyx minimalOrientationHeadingToBase_;
  EulerAnglesZyx maximalOrientationHeadingToBase_;


  //! filtered speeds [sagittal; coronal; turning]
  FilteredDouble filteredVelocities_[3];



};

} /* namespace loco */

#endif /* LOCO_MissionControlSpeedFilter_HPP_ */
