/*
 * TerrainModelSimulation.hpp
 *
 *  Created on: May 5, 2014
 *      Author: Peter Fankhauser
 */

#ifndef LOCO_TERRAINMODELSIMULATION_HPP_
#define LOCO_TERRAINMODELSIMULATION_HPP_

#include "loco/common/TerrainModelBase.hpp"
#include "TerrainBase.hpp"

namespace loco {

class TerrainModelSimulation: public TerrainModelBase {
 public:
  TerrainModelSimulation(robotTerrain::TerrainBase* terrain);
  virtual ~TerrainModelSimulation();

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

 private:

  //! Reference to terrain.
  robotTerrain::TerrainBase* terrain_;

};

} /* namespace loco */

#endif /* LOCO_TERRAINMODELSIMULATION_HPP_ */
