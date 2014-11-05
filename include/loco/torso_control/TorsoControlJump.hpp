/*
 * TorsoControlJump.hpp
 *
 *  Created on: Oct 7, 2014
 *      Author: gech
 */

#ifndef LOCO_TORSOCONTROLJUMP_HPP_
#define LOCO_TORSOCONTROLJUMP_HPP_

#include "loco/torso_control/TorsoControlBase.hpp"
#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"
#include "loco/common/TerrainModelBase.hpp"

namespace loco {

class TorsoControlJump: public TorsoControlBase
{
 public:
  TorsoControlJump(LegGroup* legs, TorsoBase* torso,  TerrainModelBase* terrain);
  virtual ~TorsoControlJump();
  virtual bool initialize(double dt);
  virtual bool advance(double dt);
  virtual bool loadParameters(const TiXmlHandle& handle);

  virtual bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t);
  virtual void setDesiredPositionOffsetInWorldFrame(const Position& positionOffsetInWorldFrame);
  virtual void setDesiredOrientationOffset(const RotationQuaternion& orientationOffset);

 protected:
  LegGroup* legs_;
  TorsoBase* torso_;
  TerrainModelBase* terrain_;
};

} /* namespace loco */

#endif /* TORSOCONTROLJUMP_HPP_ */
