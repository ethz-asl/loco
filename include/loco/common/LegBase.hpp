/*
 * LegBase.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_LEGBASE_HPP_
#define LOCO_LEGBASE_HPP_

//#include "loco/common/LegLink.hpp"

#include "loco/common/LegStateLiftOff.hpp"
#include "loco/common/LegStateTouchDown.hpp"

#include "loco/common/TypeDefs.hpp"

#include "loco/common/LegPropertiesBase.hpp"

//#include "loco/common/LegLinkGroup.hpp"

#include "loco/state_switcher/StateSwitcher.hpp"

#include <Eigen/Core>

#include <string>
#include <iostream>



namespace loco {

class LegLink;
class LegLinkGroup;

//! Base class for a leg
/*! This should be used only as a data container
 *
 */
class LegBase {
 public:
  static constexpr int nJoints_ = 3;
  static constexpr int nDofContactPoint_ = 3;
  typedef Eigen::Array<double, nJoints_, 1> JointPositions;
  typedef Eigen::Array<double, nJoints_, 1> JointVelocities;
  typedef Eigen::Array<char, nJoints_, 1> JointControlModes;
  typedef Eigen::Array<double, nJoints_, 1> JointTorques;
  typedef Eigen::Matrix<double, nDofContactPoint_, nJoints_> TranslationJacobian;

 public:
  LegBase();
  LegBase(const std::string& name, LegLinkGroup* links);
  virtual ~LegBase();

  virtual const std::string& getName() const ;

  LegLinkGroup* getLinks();

  virtual double getStancePhase() const;
  virtual double getSwingPhase() const;

  virtual double getStanceDuration() const;
  virtual double getSwingDuration() const;

  virtual bool isGrounded() const;
  virtual bool wasGrounded() const;
  virtual bool shouldBeGrounded() const;
  virtual bool isAndShouldBeGrounded() const;
  virtual bool isSlipping() const;

  virtual double getDesiredLoadFactor() const;

  virtual bool isSupportLeg() const;
  virtual bool isLosingContact() const;


  virtual void setStancePhase(double phase);
  virtual void setSwingPhase(double phase);

  virtual void setStanceDuration(double duration);
  virtual void setSwingDuration(double duration);

  virtual void setIsGrounded(bool isGrounded);
  virtual void setWasGrounded(bool wasGrounded);
  virtual void setShouldBeGrounded(bool shouldBeGrounded);
  virtual void setIsSlipping(bool isSlipping);

  virtual void setIsSupportLeg(bool isSupportLeg);

  virtual void setIsLosingContact(bool isLosingContact);

  virtual bool didTouchDownAtLeastOnceDuringStance() const;
  virtual void setDidTouchDownAtLeastOnceDuringStance(bool didTouchDownAtLeastOnceDuringStance);

  /*!
   * Change how much a leg should be loaded.
   * @param loadFactor sets the factor how much the leg should be loaded
   *        (related to the unconstrained case without user specified load
   *        factors), value in the interval [0, 1] where 0: unloaded
   *        and 1: completely loaded.
   */
  virtual void setDesiredLoadFactor(double loadFactor);

  LegStateTouchDown* getStateTouchDown();
  const LegStateTouchDown& getStateTouchDown() const;

  LegStateLiftOff* getStateLiftOff();
  const LegStateLiftOff& getStateLiftOff() const;

  virtual const Position& getWorldToFootPositionInWorldFrame() const = 0;
  virtual const Position& getWorldToHipPositionInWorldFrame() const  = 0;

  virtual const Position& getWorldToFootPositionInBaseFrame() const = 0;
  virtual const Position& getWorldToHipPositionInBaseFrame() const  = 0;

  virtual const Position& getBaseToFootPositionInBaseFrame() const = 0;
  virtual const Position& getBaseToHipPositionInBaseFrame() const = 0;

  virtual const LinearVelocity& getFootLinearVelocityInWorldFrame() const  = 0;
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
  virtual void setMeasuredJointVelocities(const JointVelocities& jointVelocities);

  virtual const JointControlModes& getDesiredJointControlModes();
  virtual const JointPositions& getDesiredJointPositions();
  virtual const JointPositions& getMeasuredJointPositions();
  virtual const JointVelocities& getMeasuredJointVelocities();
  virtual const JointTorques& getDesiredJointTorques();

  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;

  virtual LegPropertiesBase& getProperties() = 0;
  virtual const LegPropertiesBase& getProperties() const = 0;
  virtual int getId() const = 0;

  const Position& getDesiredWorldToFootPositionInWorldFrame() const;
  void setDesireWorldToFootPositionInWorldFrame(const Position& position);

	void setPreviousStancePhase(double previousStancePhase);
	double getPreviousStancePhase() const;

	void setPreviousSwingPhase(double previousSwingPhase);
	double getPreviousSwingPhase() const;

	virtual StateSwitcher* getStateSwitcher() const;

protected:
  std::string name_;

  LegLinkGroup* links_;

  double stancePhase_;
  double previousStancePhase_;

  double swingPhase_;
  double previousSwingPhase_;

  double stanceDuration_;
  double swingDuration_;

  bool isGrounded_;
  bool wasGrounded_;
  bool shouldBeGrounded_;
  bool isSlipping_;
  bool isSupportLeg_;
  bool isLosingContact_;
  bool didTouchDownAtLeastOnceDuringStance_;

  double loadFactor_;

  LegStateTouchDown stateTouchDown_;

  LegStateLiftOff stateLiftOff_;

  JointControlModes desiredJointControlModes_;
  JointPositions desiredJointPositions_;
  JointPositions measuredJointPositions_;
  JointVelocities measuredJointVelocities_;
  JointTorques desiredJointTorques_;

  Position desiredWorldToFootPositionInWorldFrame_;

  StateSwitcher* stateSwitcher_;

};

} /* namespace loco */

#endif /* LOCO_LEGBASE_HPP_ */
