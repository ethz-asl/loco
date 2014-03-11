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
 * @ingroup robotTask
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

	virtual void advance(double dt) = 0;
};

} // namespace loco

#endif /* LOCO_FOOTPLACEMENTSTRATEGYBASE_HPP_ */
