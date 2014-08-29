/*
 * TerrainModelFreePlane.hpp
 *
 *  Created on: Aug 28, 2014
 *      Author: C. Dario Bellicoso
 */

#ifndef TERRAINMODELFREEPLANE_HPP_
#define TERRAINMODELFREEPLANE_HPP_

#include "loco/common/TerrainModelBase.hpp"

namespace loco {

  class TerrainModelFreePlane: public TerrainModelBase {

   public:

      TerrainModelFreePlane();
      virtual ~TerrainModelFreePlane();

      /*! Initializes the plane at zero height and normal parallel to z-axis in world frame
       * @param dt  time step
       * @returns true
       */
      virtual bool initialize(double dt);

      /*! Gets the surface normal of the terrain at a certain position.
       * @param[in] positionWorldToLocationInWorldFrame the place to get the surface normal from (in the world frame)
       * @param[out] normalInWorldFrame the surface normal (in the world frame)
       * @return true if successful, false otherwise
       */
      //virtual bool getNormal(const loco::Position& positionWorldToLocationInWorldFrame, loco::Vector& normalInWorldFrame) const = 0;
      bool getAttitude(double &alpha, double &beta);

      virtual bool getHeight(loco::Position& positionWorldToLocationInWorldFrame) const;
      virtual bool getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const;
      virtual bool getNormal(const loco::Position& positionWorldToLocationInWorldFrame, loco::Vector& normalInWorldFrame) const;
      virtual bool getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const;

      // Set plane properties
      void setNormalAndPosition(loco::Vector normal, loco::Position position);

      /* getheightatposition */
      /* getnormal */
      /* get rollpitch */

   protected:

      double frictionCoefficientBetweenTerrainAndFoot_;

      // Plane properties
      loco::Vector normalInWorldFrame_;
      loco::Position normalPositionInWorldFrame_;

  }; // class

} /* namespace loco */


#endif /* TERRAINMODELFREEPLANE_HPP_ */
