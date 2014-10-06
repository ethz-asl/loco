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
      frictionCoefficientBetweenTerrainAndFoot_(0.6)
  {

  } // constructor


  TerrainModelFreePlane::~TerrainModelFreePlane() {

  } // destructor


  bool TerrainModelFreePlane::initialize(double dt) {
    // Initialize the plane as coincident with the ground
    normalInWorldFrame_ = loco::Vector::UnitZ();
    positionInWorldFrame_.setZero();

    frictionCoefficientBetweenTerrainAndFoot_ = 0.6;

    return true;
  } // initialize


  void TerrainModelFreePlane::setNormalandPositionInWorldFrame(const loco::Vector& normal, const loco::Position& position) {
    normalInWorldFrame_ = normal;
    positionInWorldFrame_ = position;
  } // set normal and position


  bool TerrainModelFreePlane::getNormal(const loco::Position& positionWorldToLocationInWorldFrame, loco::Vector& normalInWorldFrame) const {
    // For a plane, the normal is constant (independent from the position at which it is evaluated)
    normalInWorldFrame = normalInWorldFrame_;
    return true;
  } // get normal


  bool TerrainModelFreePlane::getHeight(loco::Position& positionWorldToLocationInWorldFrame) const {
    /* Dividing by normalInWorldFrame.z() is safe because the plane equation is z = d-ax-by.
     * If the normal is normalized, it's z component will still be greater than zero
     */
    positionWorldToLocationInWorldFrame.z() = positionInWorldFrame_.z()
                                              + normalInWorldFrame_.x()*( positionInWorldFrame_.x()-positionWorldToLocationInWorldFrame.x() )
                                              + normalInWorldFrame_.y()*( positionInWorldFrame_.y()-positionWorldToLocationInWorldFrame.y() );
    positionWorldToLocationInWorldFrame.z() /= normalInWorldFrame_.z();

    return true;
  } // get height at position, update position


  bool TerrainModelFreePlane::getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const {
    /* Dividing by normalInWorldFrame.z() is safe because the plane equation is z = d-ax-by.
     * If the normal is normalized, it's z component will still be greater than zero
     */
    heightInWorldFrame = positionInWorldFrame_.z()
                         + normalInWorldFrame_.x()*( positionInWorldFrame_.x()-positionWorldToLocationInWorldFrame.x() )
                         + normalInWorldFrame_.y()*( positionInWorldFrame_.y()-positionWorldToLocationInWorldFrame.y() );
    heightInWorldFrame /= normalInWorldFrame_.z();

    return true;
  } // get height at position, return height


  bool TerrainModelFreePlane::getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const {
    frictionCoefficient = frictionCoefficientBetweenTerrainAndFoot_;
    return true;
  } // get friction


  Position TerrainModelFreePlane::getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(const Position& positionInWorldFrame)  const {
    // For theory background see Papula, "Mathematische Formelasammlung", pag. 62
    Vector n;
    this->getNormal(positionInWorldFrame, n);
    double terrainHeightAtPosition;
    this->getHeight(positionInWorldFrame, terrainHeightAtPosition);

    Position r1, rq;
    r1 = Position::Zero();
    this->getHeight(r1);
    rq = positionInWorldFrame;
    Position r1q = rq-r1;

    double distanceFromSurfaceAlongSurfaceNormal = fabs(n.dot((Vector)r1q))/n.norm();

    if (positionInWorldFrame.z() > terrainHeightAtPosition) {
      return (positionInWorldFrame - distanceFromSurfaceAlongSurfaceNormal*Position(n));
    }
    else if (positionInWorldFrame.z() < terrainHeightAtPosition) {
      return (positionInWorldFrame + distanceFromSurfaceAlongSurfaceNormal*Position(n));
    }
    else {
      return positionInWorldFrame;
    }
  }


  double TerrainModelFreePlane::getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(const Position& positionInWorldFrame) const {
    // For theory background see Papula, "Mathematische Formelasammlung", pag. 62
    Vector n;
    this->getNormal(positionInWorldFrame, n);
    double terrainHeightAtPosition;
    this->getHeight(positionInWorldFrame, terrainHeightAtPosition);

    Position r1, rq;
    r1 = Position::Zero();
    this->getHeight(r1);
    rq = positionInWorldFrame;
    Position r1q = rq-r1;

    return fabs(n.dot((Vector)r1q))/n.norm();
  }


} /* namespace loco */


