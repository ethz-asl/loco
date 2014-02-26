/*
 * LegStarlETH.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: gech
 */

#ifndef LOCO_LEGSTARLETH_HPP_
#define LOCO_LEGSTARLETH_HPP_

#include "loco/common/LegBase.hpp"

#include <string>

namespace loco {

class LegStarlETH : public loco::LegBase {
 public:
  LegStarlETH(const std::string& name);
  virtual ~LegStarlETH();
};

} /* namespace loco */

#endif /* LEGSTARLETH_HPP_ */
