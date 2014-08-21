/*!
* @file     GaitPatternBase.hpp
* @author   Christian Gehring
* @date     Aug, 2014
* @version  1.0
* @ingroup
* @brief
*/

#include "loco/gait_pattern/GaitPatternBase.hpp"

namespace loco {

GaitPatternBase::GaitPatternBase() {

}

GaitPatternBase::~GaitPatternBase() {

}

bool GaitPatternBase::setToInterpolated(const GaitPatternBase& gaitPattern1, const GaitPatternBase& gaitPattern2, double t) {
  return false;
}

} // end namespace loco
