/*
 * TypeDefsStarlETH.hpp
 *
 *  Created on: Apr 4, 2014
 *      Author: gech
 */

#ifndef LOCO_TYPEDEFSSTARLETH_HPP_
#define LOCO_TYPEDEFSSTARLETH_HPP_

#include "loco/common/TypeDefs.hpp"

namespace loco {
typedef kindr::phys_quant::eigen_impl::Torque<double, 12> JointTorques;
typedef Eigen::Matrix<double, 19, 1> GeneralizedCoordinates;
typedef Eigen::Matrix<double, 18, 1> GeneralizedVelocities;
typedef Eigen::Matrix<double, 18, 1> GeneralizedAccelerations;

}




#endif /* LOCO_TYPEDEFSSTARLETH_HPP_ */
