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

#include <string>
#include <iostream>

namespace loco {

class LegBase {
 public:
  static constexpr int nJoints_ = 3;
  typedef Eigen::Vector3d Position;
  typedef Eigen::Array<double, nJoints_, 1> JointPositions;
  typedef Eigen::Array<char, nJoints_, 1> JointControlModes;
  typedef Eigen::Array<double, nJoints_, 1> JointTorques;
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


  virtual void setStancePhase(double phase);
  virtual void setSwingPhase(double phase);

  virtual void setStanceDuration(double duration);
  virtual void setSwingDuration(double duration);

  virtual void setIsInStanceMode(bool isInStanceMode);
  virtual void setIsInSwingMode(bool isInSwingMode);

  virtual void setIsGrounded(bool isGrounded);
  virtual void setShouldBeGrounded(bool shouldBeGrounded);

  LegStateTouchDown* getStateTouchDown();
  LegStateLiftOff* getStateLiftOff();


  virtual const Position& getFootPositionInWorldFrame() = 0;


  friend std::ostream& operator << (std::ostream& out, const LegBase& leg) {
    out << "name: " << leg.getName() << std::endl;
    out << "swing phase: " << leg.getSwingPhase() << std::endl;
    out << "stance phase: " << leg.getStancePhase() << std::endl;
    out << "swing duration: " << leg.getSwingDuration() << std::endl;
    out << "stance duration: " << leg.getStanceDuration() << std::endl;

    out << "is in stance mode: " << (leg.isInStanceMode() ? "yes" : "no") << std::endl;
    out << "is in swing mode: " << (leg.isInSwingMode() ? "yes" : "no") << std::endl;

    out << "is grounded: " << (leg.isGrounded() ? "yes" : "no") << std::endl;
    out << "should be grounded: " << (leg.shouldBeGrounded() ? "yes" : "no") << std::endl;
    return out;
  }

  virtual void setDesiredJointControlModes(const JointControlModes& jointControlMode);
  virtual void setDesiredJointPositions(const JointPositions& jointPositions);
  virtual void setDesiredJointTorques(const JointTorques& jointTorques);
  virtual const JointControlModes& getDesiredJointControlModes();
  virtual const JointPositions& getDesiredJointPositions();
  virtual const JointTorques& getDesiredJointTorques();

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

  LegStateTouchDown stateTouchDown_;
  LegStateLiftOff stateLiftOff_;

  JointControlModes desiredJointControlModes_;
  JointPositions desiredJointPositions_;
  JointTorques desiredJointTorques_;
};

} /* namespace loco */

#endif /* LOCO_LEGBASE_HPP_ */
