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
      frictionCoefficientBetweenTerrainAndFoot_(0.8),
      heightInWorldFrame_(0.0),
      heightFreePlaneInWorldFrame_(0.0)
  {

  } // constructor


  TerrainModelFreePlane::~TerrainModelFreePlane() {

  } // destructor


  bool TerrainModelFreePlane::initialize(double dt) {
    // Initialize the plane as coincident with the ground
    normalInWorldFrame_ = loco::Vector::UnitZ();
    positionInWorldFrame_.setZero();
    frictionCoefficientBetweenTerrainAndFoot_ = 0.8;
    heightInWorldFrame_ = 0.0;
    heightFreePlaneInWorldFrame_ = 0.0;
    return true;
  } // initialize





  //--- set height as in horizontal
  void TerrainModelFreePlane::setHeight(double height) {
	  heightInWorldFrame_ = height;
    setHeightFreePlane();
  }


  void TerrainModelFreePlane::setHeightFreePlane() {
	  loco::Position position;
	  position.setZero();

	  heightFreePlaneInWorldFrame_ = positionInWorldFrame_.z()
	                    		     + normalInWorldFrame_.x()*( positionInWorldFrame_.x()-position.x() )
	                                 + normalInWorldFrame_.y()*( positionInWorldFrame_.y()-position.y() );
	  heightFreePlaneInWorldFrame_ /= normalInWorldFrame_.z();
  }

  //---









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


    /*
    positionWorldToLocationInWorldFrame.z() = positionInWorldFrame_.z()
                                              + normalInWorldFrame_.x()*( positionInWorldFrame_.x()-positionWorldToLocationInWorldFrame.x() )
                                              + normalInWorldFrame_.y()*( positionInWorldFrame_.y()-positionWorldToLocationInWorldFrame.y() );
    positionWorldToLocationInWorldFrame.z() /= normalInWorldFrame_.z();
    */


    positionWorldToLocationInWorldFrame.z() = heightInWorldFrame_;

    return true;
  } // get height at position, update position


  bool TerrainModelFreePlane::getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const {


    heightInWorldFrame = heightInWorldFrame_;
    return true;

    /* Dividing by normalInWorldFrame.z() is safe because the plane equation is z = d-ax-by.
     * If the normal is normalized, it's z component will still be greater than zero
     */
    /*
    heightInWorldFrame = positionInWorldFrame_.z()
                         + normalInWorldFrame_.x()*( positionInWorldFrame_.x()-positionWorldToLocationInWorldFrame.x() )
                         + normalInWorldFrame_.y()*( positionInWorldFrame_.y()-positionWorldToLocationInWorldFrame.y() );
    heightInWorldFrame /= normalInWorldFrame_.z();
    return true;
    */
  } // get height at position, return height


  bool TerrainModelFreePlane::getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const {
    frictionCoefficient = frictionCoefficientBetweenTerrainAndFoot_;
    return true;
  } // get friction


} /* namespace loco */


