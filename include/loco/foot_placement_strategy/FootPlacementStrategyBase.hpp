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
 * @ingroup loco
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

	virtual bool advance(double dt) = 0;

	virtual bool goToStand();
	virtual bool resumeWalking();

  /*! Computes an interpolated version of the two controllers passed in as parameters.
   *  If t is 0, the current setting is set to footPlacementStrategy1, 1 -> footPlacementStrategy2, and values in between
   *  correspond to interpolated parameter set.
   * @param footPlacementStrategy1
   * @param footPlacementStrategy2
   * @param t interpolation parameter
   * @returns true if successful
   */
	virtual bool setToInterpolated(const FootPlacementStrategyBase& footPlacementStrategy1, const FootPlacementStrategyBase& footPlacementStrategy2, double t);

protected:
	bool isFirstTimeInit_;

};

} // namespace loco

#endif /* LOCO_FOOTPLACEMENTSTRATEGYBASE_HPP_ */
