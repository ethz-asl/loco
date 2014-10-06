/*!
* @file     BaseControlBase.hpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/

#ifndef LOCO_BASECONTROLBASE_HPP_
#define LOCO_BASECONTROLBASE_HPP_

#include "tinyxml.h"

namespace loco {

class TorsoControlBase {
 public:
  TorsoControlBase();
  virtual ~TorsoControlBase();
  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;
  virtual bool loadParameters(const TiXmlHandle& handle) = 0;

  /*! Computes an interpolated version of the two controllers passed in as parameters.
   *  if t is 0, the current setting is set to controller1, 1 -> controller2, and values in between
   *  correspond to interpolated parameter set.
   * @param torsoController1
   * @param torsoController2
   * @param t interpolation parameter
   * @returns true if successful
   */
  virtual bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t);

};

} /* namespace loco */

#endif /* BASECONTROLBASE_HPP_ */
