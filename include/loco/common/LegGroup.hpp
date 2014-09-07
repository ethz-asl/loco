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
  LegGroup(LegBase* leftForeLeg, LegBase* rightForeLeg, LegBase* leftHindLeg, LegBase* rightHindLeg);

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

};

} /* namespace loco */

#endif /* LOCO_LEGGROUP_HPP_ */
