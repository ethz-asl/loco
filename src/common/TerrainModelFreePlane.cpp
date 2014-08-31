/*
 * TerrainModelFreePlane.cpp
 *
 *  Created on: Aug 28, 2014
 *      Author: C. Dario Bellicoso
 */

#include "loco/common/TerrainModelFreePlane.hpp"

namespace loco {

  TerrainModelFreePlane::TerrainModelFreePlane() :
      TerrainModelBase(),
      normalPositionInWorldFrame_(loco::Vector::Zero()),
      normalInWorldFrame_(loco::Vector::UnitZ()),
      frictionCoefficientBetweenTerrainAndFoot_(0.8),
      planeEquationConstantTerm_(0.0)
  {

  } // constructor


  TerrainModelFreePlane::~TerrainModelFreePlane() {

  } // destructor


  bool TerrainModelFreePlane::initialize(double dt) {
    // Initialize the plane as coincident with the ground
    normalInWorldFrame_ = loco::Vector::UnitZ();
    normalPositionInWorldFrame_.setZero();
    frictionCoefficientBetweenTerrainAndFoot_ = 0.8;
    planeEquationConstantTerm_ = 0.0;
    return true;
  } // initialize


  void TerrainModelFreePlane::setNormalAndConstantTerm(loco::Vector normal, double constantTerm) {
    normalInWorldFrame_ = normal;
    planeEquationConstantTerm_ = constantTerm;
  } // set normal and constant term


  bool TerrainModelFreePlane::getNormal(const loco::Position& positionWorldToLocationInWorldFrame, loco::Vector& normalInWorldFrame) const {
    normalInWorldFrame = normalInWorldFrame_;
    return true;
  } // get normal


  bool TerrainModelFreePlane::getHeight(loco::Position& positionWorldToLocationInWorldFrame) const {
    positionWorldToLocationInWorldFrame.z() = planeEquationConstantTerm_
                                              - normalInWorldFrame_.x()*positionWorldToLocationInWorldFrame.x()
                                              - normalInWorldFrame_.y()*positionWorldToLocationInWorldFrame.y();
    return true;
  } // get height (position)


  bool TerrainModelFreePlane::getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const {
    heightInWorldFrame = planeEquationConstantTerm_
                         - normalInWorldFrame_.x()*positionWorldToLocationInWorldFrame.x()
                         - normalInWorldFrame_.y()*positionWorldToLocationInWorldFrame.y();
    return true;
  } // get height (position, &height)


  bool TerrainModelFreePlane::getAttitude(double &alpha, double &beta) {
    /*
    loco::EulerAnglesZyx rot;
    rot.setFromVectors(loco::Vector::UnitX(), normalInWorldFrame_);
    rot.setFromVectors(loco::Vector::UnitZ(), normalInWorldFrame_);
    */
    return true;
  } // get attitude


  bool TerrainModelFreePlane::getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const {
    frictionCoefficient = frictionCoefficientBetweenTerrainAndFoot_;
    return true;
  } // get friction


} /* namespace loco */


