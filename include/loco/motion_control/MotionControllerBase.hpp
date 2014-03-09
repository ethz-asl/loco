/*
 * MotionControllerBase.hpp
 *
 *  Created on: March 6, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef MOTIONCONTROLLERBASE_H_
#define MOTIONCONTROLLERBASE_H_

// Shared pointers
#include <memory>
// Locomotion controller common
#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"
// Contact force distribution
#include "loco/contact_force_distribution/ContactForceDistributionBase.hpp"
// Parameters
#include "tinyxml.h"
// Needed?
#include "loco/temp_helpers/Legs.hpp"

namespace loco {

class MotionControllerBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Constructor.
   */
  MotionControllerBase(std::shared_ptr<LegGroup> legs, std::shared_ptr<TorsoBase> torso);

  /*!
   * Destructor.
   */
  virtual ~MotionControllerBase();

  /*!
   * Load parameters.
   * @return true if successful
   */
  virtual bool loadParameters(const TiXmlHandle& handle) = 0;

  /*!
   * Add data to logger (optional).
   * @return true if successful
   */
  virtual bool addToLogger() = 0;

  /*!
   * Computes the joint torques from the desired base pose.
   * @return true if successful
   */
  virtual bool compute() = 0;

 protected:
  std::shared_ptr<LegGroup> legs_;
  std::shared_ptr<TorsoBase> torso_;

  //! True if parameters are successfully loaded.
  bool isParametersLoaded_;

  //! True if data is logged.
  bool isLogging_;

  /*!
   * Check if parameters are loaded.
   * @return true if parameters are loaded.
   */
  virtual bool checkIfParametersLoaded() const;
};

} /* namespace loco */
#endif /* MOTIONCONTROLLERBASE_H_ */
