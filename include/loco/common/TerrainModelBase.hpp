/*
 * TerrainModelBase.hpp
 *
 *  Created on: Apr 4, 2014
 *      Author: gech
 */

#ifndef LOCO_TERRAINMODELBASE_HPP_
#define LOCO_TERRAINMODELBASE_HPP_

#include "loco/common/TypeDefs.hpp"

namespace loco {

/*! Base class for model of the terrain used for control
 */
class TerrainModelBase {
 public:
  //! Constructor
  TerrainModelBase();
  virtual ~TerrainModelBase();

  /*! Initializes the model
   * @param dt time step
   * @returns true if initialization was successful
   */
  virtual bool initialize(double dt) = 0;

  /*! Gets the surface normal of the terrain at a certain position.
   * @param[in] positionWorldToLocationInWorldFrame the place to get the surface normal from (in the world frame)
   * @param[out] normalInWorldFrame the surface normal (in the world frame)
   * @return true if successful, false otherwise
   */
  virtual bool getNormal(const loco::Position& positionWorldToLocationInWorldFrame, loco::Vector& normalInWorldFrame) const = 0;

  /*! Gets the height of the terrain at the coordinate (positionWorldToLocationInWorldFrame(), positionWorldToLocationInWorldFrame())
   * (in world frame) and sets the position.z() as the height (in world frame).
   * @param[in/out] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location expressed in world frame
   * @return true if successful, false otherwise
   */
  virtual bool getHeight(loco::Position& positionWorldToLocationInWorldFrame) const = 0;

  /*!Gets the height of the terrain at a certain position.
   * (in world frame) and sets the position.z() as the height (in world frame).
   * @param[in] positionWorldToLocationInWorldFrame   position of the requested location expressed in world frame
   * @param[out]  height of the terrain in world frame
   * @returns true if successful, false otherwise
   */
  virtual bool getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const = 0;

  /*! Gets the friction coefficient between foot and terrain at a certain location.
   * @param positionWorldToLocationInWorldFrame   position of the requested location expressed in world frame
   * @param frictionCoefficient   friction coefficient
   * @returns true if successful, false otherwise
   */
  virtual bool getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const = 0;

};

} /* namespace loco */

#endif /* LOCO_TERRAINMODELBASE_HPP_ */
