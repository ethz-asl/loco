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
#include "kindr/vector/VectorEigen.hpp"

namespace loco {

typedef kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD Pose;
typedef kindr::poses::eigen_impl::TwistLinearVelocityLocalAngularVelocityD Twist;

typedef kindr::rotations::eigen_impl::RotationQuaternionPD RotationQuaternion;
typedef kindr::rotations::eigen_impl::AngleAxisPD AngleAxis;
typedef kindr::rotations::eigen_impl::RotationMatrixPD RotationMatrix;

typedef kindr::positions::eigen_impl::Position3D Position;

typedef kindr::positions::eigen_impl::LinearVelocityD LinearVelocity;
typedef kindr::rotations::eigen_impl::LocalAngularVelocityAD LocalAngularVelocity;

typedef kindr::vector::eigen_impl::Acceleration3D LinearAcceleration;

typedef kindr::vector::eigen_impl::Force3D Force;
typedef kindr::vector::eigen_impl::Torque3D Torque;


} // namespace loco


#endif /* LOCO_TYPEDEFS_HPP_ */
