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
      positionInWorldFrame_(loco::Vector::Zero()),
      normalInWorldFrame_(loco::Vector::UnitZ()),
      frictionCoefficientBetweenTerrainAndFoot_(0.8)
  {

  } // constructor


  TerrainModelFreePlane::~TerrainModelFreePlane() {

  } // destructor


  bool TerrainModelFreePlane::initialize(double dt) {
    // Initialize the plane as coincident with the ground
    normalInWorldFrame_ = loco::Vector::UnitZ();
    positionInWorldFrame_.setZero();
    frictionCoefficientBetweenTerrainAndFoot_ = 0.8;
    return true;
  } // initialize


  void TerrainModelFreePlane::setNormalandPositionInWorldFrame(loco::Vector& normal, loco::Position& position) {
    normalInWorldFrame_ = normal;
    positionInWorldFrame_ = position;
  } // set normal and posiition


  bool TerrainModelFreePlane::getNormal(const loco::Position& positionWorldToLocationInWorldFrame, loco::Vector& normalInWorldFrame) const {
    // for a plane, the normal is constant (independent from the position at which it is evaluated)
    normalInWorldFrame = normalInWorldFrame_;
    return true;
  } // get normal


  bool TerrainModelFreePlane::getHeight(loco::Position& positionWorldToLocationInWorldFrame) const {
    positionWorldToLocationInWorldFrame.z() = positionInWorldFrame_.z()
                                              + normalInWorldFrame_.x()*( positionInWorldFrame_.x()-positionWorldToLocationInWorldFrame.x() )
                                              + normalInWorldFrame_.y()*( positionInWorldFrame_.y()-positionWorldToLocationInWorldFrame.y() );
    return true;
  } // get height (position)


  bool TerrainModelFreePlane::getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const {
    heightInWorldFrame = positionInWorldFrame_.z()
                         + normalInWorldFrame_.x()*( positionInWorldFrame_.x()-positionWorldToLocationInWorldFrame.x() )
                         + normalInWorldFrame_.y()*( positionInWorldFrame_.y()-positionWorldToLocationInWorldFrame.y() );
    return true;
  } // get height (position, &height)


  bool TerrainModelFreePlane::getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const {
    frictionCoefficient = frictionCoefficientBetweenTerrainAndFoot_;
    return true;
  } // get friction


} /* namespace loco */


