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

#include "starlethModel/RobotModel.hpp"

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
//  LegStarlETH(const std::string& name, int iLeg, LegLinkGroup* links, robotModel::RobotModel* robotModel);
  LegStarlETH(const std::string& name, int iLeg, robotModel::RobotModel* robotModel);

  virtual ~LegStarlETH();
  virtual const Position& getPositionWorldToFootInWorldFrame()  const;
  virtual const Position& getPositionWorldToHipInWorldFrame()  const;
  virtual const LinearVelocity& getLinearVelocityHipInWorldFrame()  const;
  virtual const LinearVelocity& getLinearVelocityFootInWorldFrame()  const;

  virtual const Position& getPositionWorldToFootInBaseFrame() const;
  virtual const Position& getPositionWorldToHipInBaseFrame() const;

  virtual const Position& getPositionBaseToFootInBaseFrame() const;
  virtual const Position& getPositionBaseToHipInBaseFrame() const;

  virtual JointPositions getJointPositionsFromPositionBaseToFootInBaseFrame(const Position& positionBaseToFootInBaseFrame);

  virtual const Force& getFootContactForceInWorldFrame() const;
  virtual const Vector& getFootContactNormalInWorldFrame() const;

  virtual const TranslationJacobian& getTranslationJacobianFromBaseToFootInBaseFrame() const;


  virtual bool initialize(double dt);
  virtual bool advance(double dt);

  virtual LegPropertiesBase& getProperties();
  virtual const LegPropertiesBase& getProperties() const;

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

  TranslationJacobian translationJacobianBaseToFootInBaseFrame_;


  Force forceFootContactInWorldFrame_;
  Vector normalFootContactInWorldFrame_;

  LegPropertiesStarlETH properties_;
};

} /* namespace loco */

#endif /* LEGSTARLETH_HPP_ */
