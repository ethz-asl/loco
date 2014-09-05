/*
 * LegBase.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#include "loco/common/LegBase.hpp"

namespace loco {

LegBase::LegBase() :
    name_(""),
    links_(nullptr),
    stancePhase_(0.0),
    swingPhase_(0.0),
    stanceDuration_(0.0),
    swingDuration_(0.0),
    isInStanceMode_(false),
    wasInStanceMode_(false),
    isInSwingMode_(true),
    wasInSwingMode_(true),
    isGrounded_(false),
    shouldBeGrounded_(false),
    loadFactor_(1.0)
{

}

LegBase::LegBase(const std::string& name, LegLinkGroup* links) :
    name_(name),
    links_(links),
    stancePhase_(0.0),
    swingPhase_(0.0),
    stanceDuration_(0.0),
    swingDuration_(0.0),
    isInStanceMode_(false),
    wasInStanceMode_(false),
    isInSwingMode_(true),
    wasInSwingMode_(true),
    isGrounded_(false),
    shouldBeGrounded_(false),
    loadFactor_(1.0)
{

}

LegBase::~LegBase() {

}

LegLinkGroup* LegBase::getLinks() {
  return links_;
}


double LegBase::getStancePhase() const {
  return stancePhase_;
}
double LegBase::getSwingPhase() const {
  return swingPhase_;
}

double LegBase::getStanceDuration() const {
  return stanceDuration_;
}
double LegBase::getSwingDuration() const {
  return swingDuration_;
}

bool LegBase::isInStanceMode() const {
  return isInStanceMode_;
}
bool LegBase::isInSwingMode() const {
  return isInSwingMode_;
}

bool LegBase::wasInStanceMode() const {
  return wasInStanceMode_;
}
bool LegBase::wasInSwingMode() const {
  return wasInSwingMode_;
}


bool LegBase::isGrounded() const {
  return isGrounded_;
}
bool LegBase::shouldBeGrounded() const {
  return shouldBeGrounded_;
}
bool LegBase::isAndShouldBeGrounded() const {
  return (isGrounded_ && shouldBeGrounded_);
}

double LegBase::getDesiredLoadFactor() const
{
  return loadFactor_;
}





void LegBase::setStancePhase(double phase) {
  stancePhase_ = phase;
}

void LegBase::setSwingPhase(double phase) {
  swingPhase_ = phase;
}

void LegBase::setStanceDuration(double duration) {
  stanceDuration_ = duration;
}

void LegBase::setSwingDuration(double duration) {
  swingDuration_ = duration;
}

void LegBase::setIsInStanceMode(bool isInStanceMode) {
  isInStanceMode_ = isInStanceMode;
}

void LegBase::setIsInSwingMode(bool isInSwingMode) {
  isInSwingMode_ = isInSwingMode;
}

void LegBase::setWasInStanceMode(bool wasInStanceMode) {
  wasInStanceMode_ = wasInStanceMode;
}

void LegBase::setWasInSwingMode(bool wasInSwingMode) {
  wasInSwingMode_ = wasInSwingMode;
}


void LegBase::setIsGrounded(bool isGrounded) {
  isGrounded_ = isGrounded;
}

void LegBase::setShouldBeGrounded(bool shouldBeGrounded) {
  shouldBeGrounded_ = shouldBeGrounded;
}

void LegBase::setDesiredLoadFactor(double loadFactor)
{
  // TODO Check for validity
  loadFactor_ = loadFactor;
}


LegStateTouchDown* LegBase::getStateTouchDown()       { return &stateTouchDown_; }
LegStateTouchDown* LegBase::getStateTouchDownEarly()  { return &stateTouchDownEarly_; }
LegStateTouchDown* LegBase::getStateTouchDownLate()   { return &stateTouchDownLate_; }


LegStateLiftOff* LegBase::getStateLiftOff()       { return &stateLiftOff_; }
LegStateLiftOff* LegBase::getStateLiftOffEarly()  { return &stateLiftOffEarly_; }
LegStateLiftOff* LegBase::getStateLiftOffLate()   { return &stateLiftOffLate_; }


void LegBase::setDesiredJointPositions(const JointPositions& jointPositions)
{
  desiredJointPositions_ = jointPositions;
}

void LegBase::setMeasuredJointPositions(const JointPositions& jointPositions)
{
  measuredJointPositions_ = jointPositions;
}

void LegBase::setMeasuredJointVelocities(const JointVelocities& jointVelocities)
{
  measuredJointVelocities_ = jointVelocities;
}


void LegBase::setDesiredJointTorques(const JointTorques& jointTorques)
{
  desiredJointTorques_ = jointTorques;
}

const LegBase::JointPositions& LegBase::getDesiredJointPositions()
{
  return desiredJointPositions_;
}

const LegBase::JointPositions& LegBase::getMeasuredJointPositions()
{
  return measuredJointPositions_;
}

const LegBase::JointVelocities& LegBase::getMeasuredJointVelocities()
{
  return measuredJointVelocities_;
}

void LegBase::setDesiredJointControlModes(const JointControlModes& jointControlMode)
{
  desiredJointControlModes_ = jointControlMode;
}

const LegBase::JointControlModes& LegBase::getDesiredJointControlModes()
{
  return desiredJointControlModes_;
}

const LegBase::JointTorques& LegBase::getDesiredJointTorques()
{
  return desiredJointTorques_;
}

const std::string& LegBase::getName() const {
  return name_;
}

std::ostream& operator << (std::ostream& out, const LegBase& leg) {
  out << "name: " << leg.getName() << std::endl;
  out << "swing phase: " << leg.getSwingPhase() << std::endl;
  out << "stance phase: " << leg.getStancePhase() << std::endl;
  out << "swing duration: " << leg.getSwingDuration() << std::endl;
  out << "stance duration: " << leg.getStanceDuration() << std::endl;

  out << "is in stance mode: " << (leg.isInStanceMode() ? "yes" : "no") << std::endl;
  out << "is in swing mode: " << (leg.isInSwingMode() ? "yes" : "no") << std::endl;

  out << "was in stance mode: " << (leg.wasInStanceMode() ? "yes" : "no") << std::endl;
  out << "was in swing mode: " << (leg.wasInSwingMode() ? "yes" : "no") << std::endl;

  out << "is grounded: " << (leg.isGrounded() ? "yes" : "no") << std::endl;
  out << "should be grounded: " << (leg.shouldBeGrounded() ? "yes" : "no") << std::endl;

  return out;
}

const Position& LegBase::getDesiredWorldToFootPositionInWorldFrame() const {
  return desiredWorldToFootPositionInWorldFrame_;
}

void LegBase::setDesireWorldToFootPositionInWorldFrame(const Position& position) {
  desiredWorldToFootPositionInWorldFrame_ = position;
}



} /* namespace loco */
