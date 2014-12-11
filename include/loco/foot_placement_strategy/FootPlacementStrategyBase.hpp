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
/*!
* @file 	FootPlacementStrategyBase.hpp
* @author 	Christian Gehring, Stelian Coros
* @date		  Sep 7, 2012
* @version 	1.0
* @ingroup
* @brief
*/

#ifndef LOCO_FOOTPLACEMENTSTRATEGYBASE_HPP_
#define LOCO_FOOTPLACEMENTSTRATEGYBASE_HPP_


#include "loco/common/TypeDefs.hpp"

#include <Eigen/Core>
#include <tinyxml.h>

namespace loco {

//! Base class for foot placement algorithms
/*! Derive this class to implement different control algorithms
 * @ingroup loco
 */
class FootPlacementStrategyBase {
public:

	/*! Constructor
	 */
	FootPlacementStrategyBase();

	/*! Destructor
	 */
	virtual ~FootPlacementStrategyBase();

  virtual bool loadParameters(const TiXmlHandle& handle) = 0;
  virtual bool initialize(double dt) = 0;

	virtual bool advance(double dt) = 0;

	virtual bool goToStand();
	virtual bool resumeWalking();

  /*! Computes an interpolated version of the two controllers passed in as parameters.
   *  If t is 0, the current setting is set to footPlacementStrategy1, 1 -> footPlacementStrategy2, and values in between
   *  correspond to interpolated parameter set.
   * @param footPlacementStrategy1
   * @param footPlacementStrategy2
   * @param t interpolation parameter
   * @returns true if successful
   */
	virtual bool setToInterpolated(const FootPlacementStrategyBase& footPlacementStrategy1, const FootPlacementStrategyBase& footPlacementStrategy2, double t);

protected:
	bool isFirstTimeInit_;

};

} // namespace loco

#endif /* LOCO_FOOTPLACEMENTSTRATEGYBASE_HPP_ */
