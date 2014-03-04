/*
 * LegStarlETH.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_LEGSTARLETH_HPP_
#define LOCO_LEGSTARLETH_HPP_

#include "loco/common/LegBase.hpp"
#include "loco/common/LegPropertiesStarlETH.hpp"

#include <string>

#include "RobotModel.hpp"

namespace loco {

class LegStarlETH : public loco::LegBase {
 public:
  LegStarlETH(const std::string& name, int iLeg, robotModel::RobotModel* robotModel);
  virtual ~LegStarlETH();
  virtual const Position& getWorldToFootPositionInWorldFrame()  const;
  virtual const Position& getWorldToHipPositionInWorldFrame()  const;
  virtual const Velocity& getHipLinearVelocityInWorldFrame()  const;

  virtual const Position& getWorldToFootPositionInBaseFrame() const;
  virtual const Position& getWorldToHipPositionInBaseFrame() const;

  virtual JointPositions getJointPositionsFromBaseToFootPositionInBaseFrame(const Position& positionBaseToFootInBaseFrame);
  virtual void advance(double dt);

  virtual LegPropertiesBase& getProperties();
 private:
  int iLeg_;
  robotModel::RobotModel* robotModel_;
  Position positionWorldToFootInWorldFrame_;
  Position positionWorldToHipInWorldFrame_;
  Velocity linearVelocityHipInWorldFrame_;

  Position positionWorldToFootInBaseFrame_;
  Position positionWorldToHipInBaseFrame_;

  LegPropertiesStarlETH properties_;
};

} /* namespace loco */

#endif /* LEGSTARLETH_HPP_ */
