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

//! Leg of StarlETH
/*! This should be used only as a data container
 *
 */
class LegStarlETH : public loco::LegBase {
 public:
  /*! Cosntructor
   *
   * @param name        name of the leg
   * @param iLeg        index of the leg (only for internal usage)
   * @param robotModel  refernce to robot model
   */
  LegStarlETH(const std::string& name, int iLeg, robotModel::RobotModel* robotModel);

  virtual ~LegStarlETH();
  virtual const Position& getWorldToFootPositionInWorldFrame()  const;
  virtual const Position& getWorldToHipPositionInWorldFrame()  const;
  virtual const LinearVelocity& getHipLinearVelocityInWorldFrame()  const;
  virtual const LinearVelocity& getFootLinearVelocityInWorldFrame()  const;

  virtual const Position& getWorldToFootPositionInBaseFrame() const;
  virtual const Position& getWorldToHipPositionInBaseFrame() const;

  virtual const Position& getBaseToFootPositionInBaseFrame() const;
  virtual const Position& getBaseToHipPositionInBaseFrame() const;

  virtual JointPositions getJointPositionsFromBaseToFootPositionInBaseFrame(const Position& positionBaseToFootInBaseFrame);

  virtual const Force& getFootContactForceInWorldFrame() const;
  virtual const Vector& getFootContactNormalInWorldFrame() const;

  virtual const TranslationJacobian& getTranslationJacobianFromBaseToFootInBaseFrame() const;

  virtual bool initialize(double dt);
  virtual bool advance(double dt);

  virtual LegPropertiesBase& getProperties();

  //! Index of the leg (only for debugging)
  virtual int getId() const;

 private:
  //! index of the leg (only used to access robot model)
  int iLeg_;
  //! reference to robot model
  robotModel::RobotModel* robotModel_;

  Position positionWorldToFootInWorldFrame_;
  Position positionWorldToHipInWorldFrame_;
  LinearVelocity linearVelocityHipInWorldFrame_;
  LinearVelocity linearVelocityFootInWorldFrame_;

  Position positionWorldToFootInBaseFrame_;
  Position positionWorldToHipInBaseFrame_;

  Position positionBaseToFootInBaseFrame_;
  Position positionBaseToHipInBaseFrame_;

  Eigen::Matrix<double, nDofContactPoint_, nJoints_> translationJacobianBaseToFootInBaseFrame_;

  Force forceFootContactInWorldFrame_;
  Vector normalFootContactInWorldFrame_;

  LegPropertiesStarlETH properties_;
};

} /* namespace loco */

#endif /* LEGSTARLETH_HPP_ */
