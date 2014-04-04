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

class TerrainModelBase {
 public:
  TerrainModelBase();
  virtual ~TerrainModelBase();

  /*!
   * Gets the surface normal of the terrain at a certain position.
   * @param[in] position the place to get the surface normal from (in the world frame)
   * @param[out] normal the surface normal (in the world frame)
   * @return true if successful, false otherwise
   */
  virtual bool getNormal(const loco::Position& position, loco::Vector& normal) const = 0;

  /*!
   * Gets the height of the terrain at the coordinate (position.x(), position.y())
   * (in world frame) and sets the position.z() as the height (in world frame).
   * @param[in/out] position.z() is set as the height of the terrain at
   *                (position.x(), position.y()) (in world frame)
   * @return true if successful, false otherwise
   */
  virtual bool getHeight(loco::Position& position) const = 0;
};

} /* namespace loco */

#endif /* LOCO_TERRAINMODELBASE_HPP_ */
