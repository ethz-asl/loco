/*
 * TerrainPerceptionFreePlane.cpp
 *
 *  Created on: Aug 28, 2014
 *      Author: C. Dario Bellicoso
 */

#include "loco/terrain_perception/TerrainPerceptionFreePlane.hpp"

namespace loco {


  TerrainPerceptionFreePlane::TerrainPerceptionFreePlane(TerrainModelFreePlane* terrainModel, LegGroup* legs, TorsoStarlETH* torsoStarleth):
    TerrainPerceptionBase(),
    terrainModel_(terrainModel),
    legs_(legs),
    torsoStarleth_(torsoStarleth)
  {

  } // constructor


  TerrainPerceptionFreePlane::~TerrainPerceptionFreePlane() {

  } // desctuctor


  bool TerrainPerceptionFreePlane::initialize(double dt) {
    int legID = 0;

    // initialize foot positions
    for (auto leg: *legs_) {
      //lastFootPositionsInBaseFrame[legID] = leg->getBaseToFootPositionInBaseFrame();
      lastFootPositionsInBaseFrame[legID] = leg->getWorldToFootPositionInWorldFrame();
      legID++;
    }

    return true;

  } // initialize


  bool TerrainPerceptionFreePlane::advance(double dt) {

    // update each foot position in base frame if they are grounded and were in swing mode.
    // the update will be done only on first contact of each stance phase.
    int legID = 0;
    bool gotNewTouchDown = false;

    for (auto leg: *legs_) {
      if ( leg->getStateTouchDown()->isNow() ) {
        //lastFootPositionsInBaseFrame[legID] = leg->getBaseToFootPositionInBaseFrame();
        lastFootPositionsInBaseFrame[legID] = leg->getWorldToFootPositionInWorldFrame();
        gotNewTouchDown = true;
      }
      legID++;
    }

    if (gotNewTouchDown) {
      estimatePlane();
    }

    return true;

  } // advance


  void TerrainPerceptionFreePlane::estimatePlane() {
    // estimate the plane which best fits the most recent contact points of each foot in base frame
    // using least squares (pseudo inversion of the regressor matrix H)
    //
    // parameters       -> [a b d]^T
    // plane equation   -> z = d-ax-by
    // normal to plane  -> n = [a b 1]^T

    Eigen::Matrix<double,4,3> H;
    Eigen::Vector4d y, pi;
    loco::Vector normal;
    loco::Position position;
    double a, b, d;

    H.setZero();
    y.setZero();
    pi.setZero();
    normal.setZero();
    position.setZero();
    a = 0;
    b = 0;
    d = 0;

    //H.block<3,1>(0,0) = lastFootPositionsInBaseFrame[0];

    H << lastFootPositionsInBaseFrame[0].x(), lastFootPositionsInBaseFrame[0].y(), 1,
         lastFootPositionsInBaseFrame[1].x(), lastFootPositionsInBaseFrame[1].y(), 1,
         lastFootPositionsInBaseFrame[2].x(), lastFootPositionsInBaseFrame[2].y(), 1,
         lastFootPositionsInBaseFrame[3].x(), lastFootPositionsInBaseFrame[3].y(), 1;
    y << lastFootPositionsInBaseFrame[0].z(),
         lastFootPositionsInBaseFrame[1].z(),
         lastFootPositionsInBaseFrame[2].z(),
         lastFootPositionsInBaseFrame[3].z();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    pi = svd.solve(y);

    normal << -pi(0), -pi(1), 1.0;
    normal.normalize();

    position << pi(2)/normal.norm() *normal(0),
                pi(2)/normal.norm() *normal(1),
                pi(2)/normal.norm() *normal(2);

    RotationQuaternion worldToBaseRotationInWorldFrame;
    worldToBaseRotationInWorldFrame = torsoStarleth_->getMeasuredState().getWorldToBaseOrientationInWorldFrame();

    //normal = worldToBaseRotationInWorldFrame.inverseRotate(normal);
    //position = worldToBaseRotationInWorldFrame.inverseRotate(position);

    /*
    RotationQuaternion rot;
    normal = rot.inverseRotate(normal);
    RotationMatrix mat(rot);
    mat.matrix()
    */

    // update the terrain model
    terrainModel_->setNormalAndPosition(normal, position);

  } // estimate plane

} /* namespace loco */
