/*
 * TerrainModelSimulation.hpp
 *
 *  Created on: May 5, 2014
 *      Author: Peter Fankhauser, Christian Gehring
 */

#ifndef LOCO_TERRAINMODELSIMULATION_HPP_
#define LOCO_TERRAINMODELSIMULATION_HPP_

#include "loco/common/TerrainModelBase.hpp"
#include "robotUtils/terrains/TerrainBase.hpp"

namespace loco {

/*! This terrain model gives direct access to the simulation.
 * The model cannot be used on the real robot!
 */
class TerrainModelSimulation: public TerrainModelBase {
 public:
  TerrainModelSimulation(robotTerrain::TerrainBase* terrain);
  virtual ~TerrainModelSimulation();

  virtual bool initialize(double dt);

  /*!
   * Gets the surface normal of the terrain at a certain position.
   * @param[in] position the place to get the surface normal from (in the world frame).
   * @param[out] normal the surface normal (in the world frame).
   * @return true if successful, false otherwise.
   */
  virtual bool getNormal(const loco::Position& position, loco::Vector& normal) const;

  /*!
   * Gets the height of the terrain at the coordinate (position.x(), position.y())
   * (in world frame) and sets the position.z() as the height (in world frame).
   * @param[in/out] position.z() is set as the height of the terrain at
   *                (position.x(), position.y()) (in world frame)
   * @return true if successful, false otherwise.
   */
  virtual bool getHeight(loco::Position& position) const;

  /*!Gets the height of the terrain at the coordinate
   * (in world frame) and sets the position.z() as the height (in world frame).
   * @param[in] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location expressed in world frame
   * @param[out]  height of the terrain in world frame
   * @return true if successful, false otherwise
   */
  virtual bool getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const;

  /*! Gets the friction coefficient between foot and terrain at a certain location.
   * @param positionWorldToLocationInWorldFrame   position of the requested location expressed in world frame
   * @param frictionCoefficient   friction coefficient
   * @returns true if successful, false otherwise
   */
  virtual bool getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const;


 private:

  //! Reference to terrain.
  robotTerrain::TerrainBase* terrain_;

};

} /* namespace loco */

#endif /* LOCO_TERRAINMODELSIMULATION_HPP_ */
