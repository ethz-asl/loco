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
 * StateBase.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_STATEBASE_HPP_
#define LOCO_STATEBASE_HPP_


#include "loco/common/TypeDefs.hpp"

#include "loco/common/TorsoStateDesired.hpp"
#include "loco/common/TorsoStateMeasured.hpp"
#include "loco/common/TorsoPropertiesBase.hpp"
namespace loco {

//! Base class for a torso
/*! This should be used only as a data container
 *
 */
class TorsoBase {
 public:
  TorsoBase();
  virtual ~TorsoBase();

  virtual TorsoStateMeasured& getMeasuredState() = 0;
  virtual TorsoStateDesired& getDesiredState() = 0;
  virtual const TorsoStateDesired& getDesiredState() const = 0;
  virtual TorsoPropertiesBase& getProperties() = 0;

  virtual double getStridePhase() = 0;
  virtual void setStridePhase(double stridePhase) = 0;

  virtual bool initialize(double dt) = 0;

  /*! Advances in time, i.e. updates states
   * @param dt time step between updates
   */
  virtual bool advance(double dt) = 0;


  friend std::ostream& operator << (std::ostream& out, const TorsoBase& torso);

};

} /* namespace loco */

#endif /* STATEBASE_HPP_ */
