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
      frictionCoefficientBetweenTerrainAndFoot_(0.8)
  {

  } // constructor


  TerrainModelFreePlane::~TerrainModelFreePlane() {

  } // destructor


  bool TerrainModelFreePlane::initialize(double dt) {
    // Initialize the plane as coincident with the ground
    normalInWorldFrame_ = loco::Vector::UnitZ();
    normalPositionInWorldFrame_.setZero();
    frictionCoefficientBetweenTerrainAndFoot_ = 0.8;
    return true;
  } // initialize


  void TerrainModelFreePlane::setNormalAndPosition(loco::Vector normal, loco::Position position) {
    normalPositionInWorldFrame_ = position;
    normalInWorldFrame_ = normal;
  } // set normal and position


  bool TerrainModelFreePlane::getNormal(const loco::Position& positionWorldToLocationInWorldFrame, loco::Vector& normalInWorldFrame) const {
    normalInWorldFrame = normalInWorldFrame_;
    return true;
  } // get normal


  bool TerrainModelFreePlane::getHeight(loco::Position& positionWorldToLocationInWorldFrame) const {
    double d = normalInWorldFrame_.dot(normalPositionInWorldFrame_);
    positionWorldToLocationInWorldFrame.z() = d - normalInWorldFrame_.x()*positionWorldToLocationInWorldFrame.x()
                                                - normalInWorldFrame_.y()*positionWorldToLocationInWorldFrame.y();
    return true;
  }


  bool TerrainModelFreePlane::getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const {
    double d = normalInWorldFrame_.dot(normalPositionInWorldFrame_);
    heightInWorldFrame = d - normalInWorldFrame_.x()*positionWorldToLocationInWorldFrame.x()
                           - normalInWorldFrame_.y()*positionWorldToLocationInWorldFrame.y();
    return true;
  }


  bool TerrainModelFreePlane::getAttitude(double &alpha, double &beta) {
    double x = normalInWorldFrame_.x();
    double y = normalInWorldFrame_.y();
    double z = normalInWorldFrame_.z();

    alpha = atan2(z, x);
    beta  = atan2(z, y);

    //EulerAngle rot;
    //rot.setFromVectors()

    return true;
  } // get attitude


  bool TerrainModelFreePlane::getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const {
    frictionCoefficient = frictionCoefficientBetweenTerrainAndFoot_;
    return true;
  } // get friction


} /* namespace loco */


