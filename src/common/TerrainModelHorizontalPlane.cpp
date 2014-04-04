/*
 * TerrainModelHorizontalPlane.cpp
 *
 *  Created on: Apr 4, 2014
 *      Author: gech
 */

#include "loco/common/TerrainModelHorizontalPlane.hpp"

namespace loco {

TerrainModelHorizontalPlane::TerrainModelHorizontalPlane() {


}

TerrainModelHorizontalPlane::~TerrainModelHorizontalPlane() {

}

bool TerrainModelHorizontalPlane::getNormal(const loco::Position& position, loco::Vector& normal) const
{
  normal = loco::Vector::UnitZ();
  return true;
}

bool TerrainModelHorizontalPlane::getHeight(loco::Position& position) const
{
  position.z() = 0.0;
  return false;
}

} /* namespace loco */
