/*
 * TerrainModelHorizontalPlane.hpp
 *
 *  Created on: Apr 4, 2014
 *      Author: gech
 */

#ifndef LOCO_TERRAINMODELHORIZONTALPLANE_HPP_
#define LOCO_TERRAINMODELHORIZONTALPLANE_HPP_

#include "loco/common/TerrainModelBase.hpp"

namespace loco {

class TerrainModelHorizontalPlane: public TerrainModelBase {
 public:
  TerrainModelHorizontalPlane();
  virtual ~TerrainModelHorizontalPlane();

  /*! Initializes the plane at zero height
   * @param dt  time step
   * @returns true
   */
  virtual bool initialize(double dt);

  /*! Gets the surface normal of the terrain at a certain position.
   * @param[in] positionWorldToLocationInWorldFrame the place to get the surface normal from (in the world frame)
   * @param[out] normalInWorldFrame the surface normal (in the world frame)
   * @return true if successful, false otherwise
   */
  virtual bool getNormal(const loco::Position& positionWorldToLocationInWorldFrame, loco::Vector& normalInWorldFrame) const;

  /*! Gets the height of the terrain at the coordinate (positionWorldToLocationInWorldFrame(), positionWorldToLocationInWorldFrame())
   * (in world frame) and sets the position.z() as the height (in world frame).
   * @param[in/out] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location expressed in world frame
   * @return true if successful, false otherwise
   */
  virtual bool getHeight(loco::Position& positionWorldToLocationInWorldFrame) const;

  /*!Gets the height of the terrain at the coordinate
   * (in world frame) and sets the position.z() as the height (in world frame).
   * @param[in] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location expressed in world frame
   * @param[out]  height of the terrain in world frame
   * @return true if successful, false otherwise
   */
  virtual bool getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const;

  /*! Sets the height of the plane expressed in world frame
   *
   * @param heightInWorldFrame
   */
  void setHeight(double heightInWorldFrame);


 protected:
  //! Height of the horizontal plane expressed in world frame
  double heightInWorldFrame_;
  //! Normal vector of the horizontal plane expressed in world frame
  loco::Vector normalInWorldFrame_;

};

} /* namespace loco */

#endif /* LOCO_TERRAINMODELHORIZONTALPLANE_HPP_ */
