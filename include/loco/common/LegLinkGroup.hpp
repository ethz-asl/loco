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
