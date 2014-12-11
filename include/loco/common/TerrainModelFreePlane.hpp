/*****************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Christian Gehring, Péter Fankhauser, C. Dario Bellicoso, Stelian Coros
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*
 * TerrainModelFreePlane.hpp
 *
 *  Created on: Aug 28, 2014
 *      Author: C. Dario Bellicoso
 */

#ifndef LOCO_TERRAINMODELFREEPLANE_HPP_
#define LOCO_TERRAINMODELFREEPLANE_HPP_

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
      virtual bool getNormal(const loco::Position& positionWorldToLocationInWorldFrame, loco::Vector& normalInWorldFrame) const;

      /*! Gets the height of the terrain at the coordinate (positionWorldToLocationInWorldFrame(), positionWorldToLocationInWorldFrame())
       * (in world frame) and sets the position.z() as the height (in world frame).
       * @param[in/out] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location expressed in world frame
       * @return true if successful, false otherwise
       */
      virtual bool getHeight(loco::Position& positionWorldToLocationInWorldFrame) const;

      /*! Gets the height of the terrain at the coordinate (positionWorldToLocationInWorldFrame(), positionWorldToLocationInWorldFrame())
       * (in world frame) and sets heightInWorldFrame as the height (in world frame).
       * @param[in] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location expressed in world frame
       * @param[out] heightInWorldFrame   height in world frame evaluated at position positionWorldToLocationInWorldFrame
       * @return true if successful, false otherwise
       */
      virtual bool getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const;

      /*! Return friction coefficient for a foot at a certain position.
       * @param[in] positionWorldToLocationInWorldFrame position from origin of world frame to the requested location expressed in world frame
       * @param[out] frictionCoefficient friction coefficient evaluated at the given position
       * @return true if successful, false otherwise
       */
      virtual bool getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const;

      /*! Set free plane parameters.
       * @param normal Normal vector to plane in world frame
       * @param position Point on plane in world frame
       */
      void setNormalandPositionInWorldFrame(const loco::Vector& normal, const loco::Position& position);

      Position getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(const Position& position)  const;
      double getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(const Position& positionInWorldFrame) const;

   public:
      double frictionCoefficientBetweenTerrainAndFoot_;

      // Plane properties
      loco::Vector normalInWorldFrame_;       // normal to the plane
      loco::Position positionInWorldFrame_;   // position of a point on the plane

  }; // class

} /* namespace loco */


#endif /* LOCO_TERRAINMODELFREEPLANE_HPP_ */
