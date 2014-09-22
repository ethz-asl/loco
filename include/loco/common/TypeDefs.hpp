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
