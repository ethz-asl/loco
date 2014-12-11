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
 * LegLinkGroup.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_LEGLINKGROUP_HPP_
#define LOCO_LEGLINKGROUP_HPP_


#include "loco/common/LegLink.hpp"
#include <vector>

namespace loco {


class LegLinkGroup {
 private:
  //! Container type
  typedef std::vector<LegLink*> LegLinks;
 public:
  typedef typename LegLinks::size_type size_type;
  typedef LegLinks::iterator iterator;
  typedef LegLinks::const_iterator const_iterator;
  typedef LegLinks::const_reference const_reference;
  typedef LegLinks::reference reference;

 private:
  //! Container of the LegLinks
  std::vector<LegLink*> legLinks_;

 public:

  /*! Constructor
   */
  LegLinkGroup();

  //! Destructor
  virtual ~LegLinkGroup();


  iterator begin() {
    return legLinks_.begin();
  }

  const_iterator begin() const {
    return legLinks_.begin();
  }

  iterator end() {
    return legLinks_.end();
  }

  const_iterator end() const {
    return legLinks_.end();
  }


  const_reference back() const {
    return legLinks_.back();
  }

  reference back() {
    return legLinks_.back();
  }

  const_reference front() const {
    return legLinks_.front();
  }

  reference front() {
    return legLinks_.front();
  }

  //! @returns number of LegLinks
  size_type size() const {
    return legLinks_.size();
  }

  /*! Adds a leg to the container
   *
   * @param leg
   */
  void addLegLink(LegLink* leg) {
    legLinks_.push_back(leg);
  }


  /*! Gets leg by index
   *
   * @param offset  index
   * @return  reference to leg
   */
  const LegLink* getLegLink(size_type offset) const {
    return legLinks_[offset];
  }

  LegLink* getLegLink(size_type offset) {
    return legLinks_[offset];
  }

};

} /* namespace loco */

#endif /* LOCO_LegLinkGroup_HPP_ */
