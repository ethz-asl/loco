/*
 * TerrainPerceptionBase.hpp
 *
 *  Created on: Apr 15, 2014
 *      Author: gech
 */

#ifndef LOCO_TERRAINPERCEPTIONBASE_HPP_
#define LOCO_TERRAINPERCEPTIONBASE_HPP_

namespace loco {

class TerrainPerceptionBase {
 public:
  TerrainPerceptionBase();
  virtual ~TerrainPerceptionBase();

  virtual bool initialize(double dt) = 0;

  /*! Advance in time
   * @param dt  time step [s]
   */
  virtual bool advance(double dt) = 0;
};

} /* namespace loco */

#endif /* LOCO_TERRAINPERCEPTIONBASE_HPP_ */
