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
      heightFreePlaneInWorldFrame_(0.0),
      filterHeightHorizontalTimeConstant_(1.0),
      filterHeightFreeTimeConstant_(0.2 ),
      heightNoise_(0.0)
  {

    filterNormalX_ = FirstOrderFilter();
    filterNormalY_ = FirstOrderFilter();
    filterNormalZ_ = FirstOrderFilter();
    filterPositionX_ = FirstOrderFilter();
    filterPositionY_ = FirstOrderFilter();
    filterPositionZ_ = FirstOrderFilter();

    filterHeightFree_ = FirstOrderFilter();
    filterHeightHorizontal_ = FirstOrderFilter();

  } // constructor


  TerrainModelFreePlane::~TerrainModelFreePlane() {

  } // destructor


  bool TerrainModelFreePlane::initialize(double dt) {
    // Initialize the plane as coincident with the ground
    normalInWorldFrame_ = loco::Vector::UnitZ();
    positionInWorldFrame_.setZero();

    normalInWorldFrameFilterInput_ = normalInWorldFrame_;
    positionInWorldFrameFilterInput_ = positionInWorldFrame_;

    frictionCoefficientBetweenTerrainAndFoot_ = 0.8;
    heightInWorldFrame_ = 0.0;
    heightFreePlaneInWorldFrame_ = 0.0;
    filterHeightHorizontalTimeConstant_ = 1.0;
    filterHeightFreeTimeConstant_ = 0.2;


    filterNormalX_.initialize(normalInWorldFrame_.x(), 0.1, 1.0);
    filterNormalY_.initialize(normalInWorldFrame_.y(), 0.1, 1.0);
    filterNormalZ_.initialize(normalInWorldFrame_.z(), 0.1, 1.0);

    filterPositionX_.initialize(positionInWorldFrame_.x(), 0.1, 1.0);
    filterPositionY_.initialize(positionInWorldFrame_.y(), 0.1, 1.0);
    filterPositionZ_.initialize(positionInWorldFrame_.z(), 0.1, 1.0);

    //filterHeightFree_.initialize(0.0, filterHeightFreeTimeConstant_, 1.0);
    //filterHeightHorizontal_.initialize(0.0, filterHeightHorizontalTimeConstant_, 1.0);

    heightNoise_ = 0.0;

    return true;
  } // initialize


  void TerrainModelFreePlane::setHeightNoise(double height) {
    heightNoise_ = height;
  }


  void TerrainModelFreePlane::advance(double dt) {
    normalInWorldFrame_.x() = filterNormalX_.advance(dt, normalInWorldFrameFilterInput_.x());
    normalInWorldFrame_.y() = filterNormalY_.advance(dt, normalInWorldFrameFilterInput_.y());
    normalInWorldFrame_.z() = filterNormalZ_.advance(dt, normalInWorldFrameFilterInput_.z());

    positionInWorldFrame_.x() = filterPositionX_.advance(dt, positionInWorldFrameFilterInput_.x());
    positionInWorldFrame_.y() = filterPositionY_.advance(dt, positionInWorldFrameFilterInput_.y());
    positionInWorldFrame_.z() = filterPositionZ_.advance(dt, positionInWorldFrameFilterInput_.z());

	  //filterHeightFree_.setContinuousTimeConstant(filterHeightFreeTimeConstant_);

    //heightFreePlaneInWorldFrame_ = filterHeightFree_.advance(dt, heightFreePlaneInWorldFrame_);
//    heightInWorldFrame_ = filterHeightHorizontal_.advance(dt, heightInWorldFrame_);


    //std::cout << "height free: " << heightFreePlaneInWorldFrame_ << " height horz: " << heightInWorldFrame_ << std::endl;

  }


  //--- set height as in horizontal
  void TerrainModelFreePlane::setHeight(double height, const loco::Position& torsoPositionInWorldFrame) {
	  heightInWorldFrame_ = height;
	  setHeightFreePlane(torsoPositionInWorldFrame);
  }


  void TerrainModelFreePlane::setHeightFreePlane(const loco::Position& torsoPositionInWorldFrame) {
	  /*
	  heightFreePlaneInWorldFrame_ = positionInWorldFrame_.z()
	                                  + normalInWorldFrame_.x()*( positionInWorldFrame_.x()-position.x() )
	                                  + normalInWorldFrame_.y()*( positionInWorldFrame_.y()-position.y() );
	  heightFreePlaneInWorldFrame_ /= normalInWorldFrame_.z();
	  */

	  getHeight(torsoPositionInWorldFrame, heightFreePlaneInWorldFrame_);

  }
  //---


  void TerrainModelFreePlane::setNormalandPositionInWorldFrame(const loco::Vector& normal, const loco::Position& position) {
    normalInWorldFrameFilterInput_ = normal;
    positionInWorldFrameFilterInput_ = position;
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


    //positionWorldToLocationInWorldFrame.z() += heightNoise_;

    //positionWorldToLocationInWorldFrame.z() = heightInWorldFrame_;
    //positionWorldToLocationInWorldFrame.z() = heightFreePlaneInWorldFrame_;
    return true;
  } // get height at position, update position


  bool TerrainModelFreePlane::getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const {


	//heightInWorldFrame = heightInWorldFrame_;
	  //heightInWorldFrame = heightFreePlaneInWorldFrame_;
    //return true;

    /* Dividing by normalInWorldFrame.z() is safe because the plane equation is z = d-ax-by.
     * If the normal is normalized, it's z component will still be greater than zero
     */

    heightInWorldFrame = positionInWorldFrame_.z()
                         + normalInWorldFrame_.x()*( positionInWorldFrame_.x()-positionWorldToLocationInWorldFrame.x() )
                         + normalInWorldFrame_.y()*( positionInWorldFrame_.y()-positionWorldToLocationInWorldFrame.y() );
    heightInWorldFrame /= normalInWorldFrame_.z();

    //heightInWorldFrame += heightNoise_;

    return true;
  } // get height at position, return height


  bool TerrainModelFreePlane::getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const {
    frictionCoefficient = frictionCoefficientBetweenTerrainAndFoot_;
    return true;
  } // get friction


} /* namespace loco */


