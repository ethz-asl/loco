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

class LegGroup {
 private:
  typedef std::vector<LegBase*> Legs;
 public:
  typedef typename Legs::size_type size_type;
  typedef Legs::iterator iterator;
  typedef Legs::const_iterator const_iterator;
  typedef Legs::const_reference const_reference;
  typedef Legs::reference reference;

 private:
  std::vector<LegBase*>  legs_;


 public:
  LegGroup();
  virtual ~LegGroup();

  LegBase* getLeftForeLeg() {
    return legs_[0];
  }
  LegBase* getRightForeLeg() {
    return legs_[1];
  }
  LegBase* getLeftHindLeg() {
    return legs_[2];
  }
  LegBase* getRightHindLeg() {
    return legs_[3];
  }

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


  size_type size() const {
    return legs_.size();
  }

  void addLeg(LegBase* leg) {
    legs_.push_back(leg);
  }


  const LegBase* getLeg(size_type offset) const {
    return legs_[offset];
  }





};

} /* namespace loco */

#endif /* LOCO_LEGGROUP_HPP_ */
