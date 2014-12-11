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
 * TypeDefs.hpp
 *
 *  Created on: Feb 27, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef LOCO_TYPEDEFS_HPP_
#define LOCO_TYPEDEFS_HPP_

#include "kindr/poses/PoseEigen.hpp"
#include "kindr/poses/PoseDiffEigen.hpp"
#include "kindr/phys_quant/PhysicalQuantitiesEigen.hpp"


namespace loco {

typedef kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD Pose;
typedef kindr::poses::eigen_impl::TwistLinearVelocityLocalAngularVelocityPD Twist;

typedef kindr::rotations::eigen_impl::RotationQuaternionPD RotationQuaternion;
typedef kindr::rotations::eigen_impl::AngleAxisPD AngleAxis;
typedef kindr::rotations::eigen_impl::RotationMatrixPD RotationMatrix;
typedef kindr::rotations::eigen_impl::EulerAnglesZyxPD EulerAnglesZyx;
typedef kindr::rotations::eigen_impl::RotationVectorPD RotationVector;

typedef kindr::phys_quant::eigen_impl::Position3D Position;

typedef kindr::phys_quant::eigen_impl::Velocity3D LinearVelocity;
typedef kindr::rotations::eigen_impl::LocalAngularVelocityPD LocalAngularVelocity;

typedef kindr::phys_quant::eigen_impl::Acceleration3D LinearAcceleration;
typedef kindr::phys_quant::eigen_impl::AngularAcceleration3D AngularAcceleration;

typedef kindr::phys_quant::eigen_impl::Force3D Force;
typedef kindr::phys_quant::eigen_impl::Torque3D Torque;

typedef kindr::phys_quant::eigen_impl::VectorTypeless3D Vector;


} // namespace loco


#endif /* LOCO_TYPEDEFS_HPP_ */
