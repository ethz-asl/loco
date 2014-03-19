/*
 * LegBase.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_LEGBASE_HPP_
#define LOCO_LEGBASE_HPP_

#include "loco/common/LegStateLiftOff.hpp"
#include "loco/common/LegStateTouchDown.hpp"

#include "loco/common/TypeDefs.hpp"
#include "loco/common/LegPropertiesBase.hpp"

#include <Eigen/Core>

#include <string>
#include <iostream>

namespace loco {

//! Base class for a leg
/*! This should be used only as a data container
 *
 */
class LegBase {
 public:
  static constexpr int nJoints_ = 3;
  static constexpr int nDofContactPoint_ = 3;
  typedef Eigen::Array<double, nJoints_, 1> JointPositions;
  typedef Eigen::Array<char, nJoints_, 1> JointControlModes;
  typedef Eigen::Array<double, nJoints_, 1> JointTorques;
  typedef Eigen::Matrix<double, nDofContactPoint_, nJoints_> TranslationJacobian;

 public:
  LegBase();
  LegBase(const std::string& name);
  virtual ~LegBase();

  virtual const std::string& getName() const ;

  virtual double getStancePhase() const;
  virtual double getSwingPhase() const;

  virtual double getStanceDuration() const;
  virtual double getSwingDuration() const;

  virtual bool isInStanceMode() const;
  virtual bool isInSwingMode() const;

  virtual bool isGrounded() const;
  virtual bool shouldBeGrounded() const;
  virtual bool isAndShouldBeGrounded() const;

  virtual double getDesiredLoadFactor() const;


  virtual void setStancePhase(double phase);
  virtual void setSwingPhase(double phase);

  virtual void setStanceDuration(double duration);
  virtual void setSwingDuration(double duration);

  virtual void setIsInStanceMode(bool isInStanceMode);
  virtual void setIsInSwingMode(bool isInSwingMode);

  virtual void setIsGrounded(bool isGrounded);
  virtual void setShouldBeGrounded(bool shouldBeGrounded);

  /*!
   * Change how much a leg should be loaded.
   * @param loadFactor sets the factor how much the leg should be loaded
   *        (related to the unconstrained case without user specified load
   *        factors), value in the interval [0, 1] where 0: unloaded
   *        and 1: completely loaded.
   */
  virtual void setDesiredLoadFactor(double loadFactor);

  virtual const Vector& getSurfaceNormal() const;
  virtual void setSurfaceNormal(const Vector& surfaceNormal) = 0;

  LegStateTouchDown* getStateTouchDown();
  LegStateLiftOff* getStateLiftOff();

  virtual const Position& getWorldToFootPositionInWorldFrame() const = 0;
  virtual const Position& getWorldToHipPositionInWorldFrame() const  = 0;

  virtual const Position& getWorldToFootPositionInBaseFrame() const = 0;
  virtual const Position& getWorldToHipPositionInBaseFrame() const  = 0;

  virtual const Position& getBaseToFootPositionInBaseFrame() const = 0;
  virtual const Position& getBaseToHipPositionInBaseFrame() const = 0;

  virtual const LinearVelocity& getHipLinearVelocityInWorldFrame() const  = 0;
  virtual JointPositions getJointPositionsFromBaseToFootPositionInBaseFrame(const Position& positionBaseToFootInBaseFrame) = 0;

  virtual const TranslationJacobian& getTranslationJacobianFromBaseToFootInBaseFrame() const = 0;

  virtual const Force& getFootContactForceInWorldFrame() const = 0;
  virtual const Vector& getFootContactNormalInWorldFrame() const = 0;

  friend std::ostream& operator << (std::ostream& out, const LegBase& leg);

  virtual void setDesiredJointControlModes(const JointControlModes& jointControlMode);
  virtual void setDesiredJointPositions(const JointPositions& jointPositions);
  virtual void setDesiredJointTorques(const JointTorques& jointTorques);
  virtual void setMeasuredJointPositions(const JointPositions& jointPositions);

  virtual const JointControlModes& getDesiredJointControlModes();
  virtual const JointPositions& getDesiredJointPositions();
  virtual const JointPositions& getMeasuredJointPositions();
  virtual const JointTorques& getDesiredJointTorques();

  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;

  virtual LegPropertiesBase& getProperties() = 0;

 protected:
  std::string name_;

  double stancePhase_;
  double swingPhase_;
  double stanceDuration_;
  double swingDuration_;

  bool isInStanceMode_;
  bool isInSwingMode_;
  bool isGrounded_;
  bool shouldBeGrounded_;

  double loadFactor_;

  Vector surfaceNormal_;

  LegStateTouchDown stateTouchDown_;
  LegStateLiftOff stateLiftOff_;

  JointControlModes desiredJointControlModes_;
  JointPositions desiredJointPositions_;
  JointPositions measuredJointPositions_;
  JointTorques desiredJointTorques_;
};

} /* namespace loco */

#endif /* LOCO_LEGBASE_HPP_ */
