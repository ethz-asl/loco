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

  /*! Sets the parameters to the interpolated ones between motionController1 and controller2.
   * @param motionController1     If the interpolation parameter is 0, then the parameter set is equal to the one of motionController1.
   * @param motionController2     If the interpolation parameter is 1, then the parameter set is equal to the one of motionController2.
   * @param t                     interpolation parameter in [0, 1]
   * @return                      true if successful
   */
  virtual bool setToInterpolated(const MotionControllerBase& motionController1, const MotionControllerBase& motionController2, double t);

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
