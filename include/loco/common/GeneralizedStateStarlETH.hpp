/*
 * GeneralizedStateStarlETH.hpp
 *
 *  Created on: Apr 4, 2014
 *      Author: gech
 */

#ifndef LOCO_GENERALIZEDSTATESTARLETH_HPP_
#define LOCO_GENERALIZEDSTATESTARLETH_HPP_

#include "loco/common/GeneralizedStateBase.hpp"
#include "loco/common/TypeDefsStarlETH.hpp"

#include <tinyxml.h>

namespace loco {

class GeneralizedStateStarlETH: public GeneralizedStateBase {
 public:
  GeneralizedStateStarlETH();
  virtual ~GeneralizedStateStarlETH();

  const GeneralizedCoordinates& getGeneralizedCoordinates() const;
  const GeneralizedVelocities& getGeneralizedVelocities() const;
  const GeneralizedAccelerations& getGeneralizedAccelerations() const;
  void setJointPositionsSymmetricToLeftForeLeg(double angleHAA, double angleHFE, double angleKFE);
  void setJointVelocitiesSymmetricToLeftForeLeg(double velocityHAA, double velocityHFE, double velocityKFE);
  void setJointPositionsForLeg(int iLeg, const Eigen::Vector3d& angles);
  bool loadParameters(const TiXmlHandle& handle);
 protected:
  GeneralizedCoordinates generalizedCoordinates_;
  GeneralizedVelocities generalizedVelocities_;
  GeneralizedAccelerations generalizedAccelerations_;


};

} /* namespace loco */

#endif /* LOCO_GENERALIZEDSTATESTARLETH_HPP_ */
