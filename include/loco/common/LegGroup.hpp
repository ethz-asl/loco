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
 * LegGroup.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_LEGGROUP_HPP_
#define LOCO_LEGGROUP_HPP_

#include "loco/common/LegBase.hpp"

#include <vector>

namespace loco {

//! Container of legs for easy access
/*! Allows to iterate over all legs:
 *  LegGroup* legs = new LegGroup();
 *  leg* leg = new LegBase()
 *  legs.addleg(leg);
 *  legs.addLeg()
 *  for (auto leg : *legs) {
 *    leg->...
 *  }
 */
class LegGroup {
 private:
  //! Container type
  typedef std::vector<LegBase*> Legs;
 public:
  typedef typename Legs::size_type size_type;
  typedef Legs::iterator iterator;
  typedef Legs::const_iterator const_iterator;
  typedef Legs::const_reference const_reference;
  typedef Legs::reference reference;

 private:
  //! Container of the legs
  std::vector<LegBase*> legs_;

 public:
  //! Constructor
  LegGroup();

  /*! Constructor
   *
   * Create leg group with four legs
   * @param leftForeLeg
   * @param rightForeLeg
   * @param leftHindLeg
   * @param rightHindLeg
   */
  LegGroup(LegBase* leftForeLeg, LegBase* rightForeLeg, LegBase* leftHindLeg,
           LegBase* rightHindLeg);

  //! Destructor
  virtual ~LegGroup();

  LegBase* getLeftForeLeg();
  LegBase* getRightForeLeg();
  LegBase* getLeftHindLeg();
  LegBase* getRightHindLeg();

  iterator begin() {
    return legs_.begin();
  }

  const_iterator begin() const {
    return legs_.begin();
  }

  iterator end() {
    return legs_.end();
  }

  const_iterator end() const {
    return legs_.end();
  }

  const_reference back() const {
    return legs_.back();
  }

  reference back() {
    return legs_.back();
  }

  const_reference front() const {
    return legs_.front();
  }

  reference front() {
    return legs_.front();
  }

  //! @returns number of legs
  size_type size() const {
    return legs_.size();
  }

  /*! Adds a leg to the container
   *
   * @param leg
   */
  void addLeg(LegBase* leg) {
    legs_.push_back(leg);
  }

  /*! Gets leg by index
   *
   * @param offset  index
   * @return  reference to leg
   */
  const LegBase* getLeg(size_type offset) const {
    return legs_[offset];
  }


  /*! Gets leg by identifier
   *
   * @param offset  index
   * @return  reference to leg
   */
  const LegBase* getLegById(int legId) const {
    for (auto leg : legs_) {
      if (leg->getId() == legId) {
        return leg;
      }
    }
    return nullptr;
  }

  /*! Gets leg by identifier
    *
    * @param offset  index
    * @return  reference to leg
    */
   LegBase* getLegById(int legId) {
     for (auto leg : legs_) {
       if (leg->getId() == legId) {
         return leg;
       }
     }
     return nullptr;
   }


};

} /* namespace loco */

#endif /* LOCO_LEGGROUP_HPP_ */
