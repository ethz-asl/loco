/*!
* @file 	  CoMOverSupportPolygonControl.hpp
* @author 	Christian Gehring, Stelian Coros, Nina Sauthoff
* @date		  Jul 17, 2012
* @version 	1.0
* @ingroup
* @brief
*/

#ifndef LOCO_COMOVERSUPPORTPOLYGONCONTROL_HPP_
#define LOCO_COMOVERSUPPORTPOLYGONCONTROL_HPP_

#include "tinyxml.h"

#include <Eigen/Core>
#include "loco/common/LegGroup.hpp"

namespace loco {

//! Support Polygon Task
/*! To maintain balance, we want to keep the projected CoM within the support polygon.
 * This class computes the error vector from the center of all feet to the desired weighted location of the center of all feet in world coordinates.
 * The error vector can then be used by a virtual force controller.
 */
class CoMOverSupportPolygonControl {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW


	//! Constructor
	CoMOverSupportPolygonControl(LegGroup* legs);

	//! Destructor
	virtual ~CoMOverSupportPolygonControl();

	/*! Gets the error vector from the center of all feet to the desired weighted location of the center of all feet in world coordinates
	 * @param legs	references to the legs
	 * @return error vector expressed in world frame
	 */
	Position getPositionErrorVectorInWorldFrame();

	/*! Loads the parameters from the XML object
	 * @param hParameterSet		handle
	 * @return	true if all parameters could be loaded
	 */
	bool loadParametersFromXML(TiXmlHandle &hParameterSet);

	/*! Stores the current paramters in the XML object
	 * @param hParameterSet		handle
	 * @return	true if all parameters could be loaded
	 */
	bool saveParametersInXML(TiXmlHandle &hParameterSet);

	/*! Computes an interpolated version of the two support polygon settings passed in as parameters.
	 *	if t is 0, the current setting is set to supportPolygon1, 1 -> supportPolygon2, and values in between
	 *	correspond to interpolated parameter set.
	 * @param supportPolygon1
	 * @param supportPolygon2
	 * @param t		interpolation parameter
	 * @return	true if successful
	 */
	bool setToInterpolated(const CoMOverSupportPolygonControl& supportPolygon1, const CoMOverSupportPolygonControl& supportPolygon2, double t);


protected:
	  LegGroup* legs_;
    //! this is the minimum weight any leg can have... if this is zero,then the COM will try to be right at center of the support polygon [0,1]
    double spMinLegWeight;

    //! this is the point in the stance phase when the body should start shifting away from the leg... when the stance phase is 1, the leg weight will be minimum
    double spStPShiftAwayFromLegStart;

    //! this is the point in the swing phase when the body should start shifting back towards the leg... when the swing phase is 1, the weight is back full on the leg
    double spSwPShiftTowardsLegStart;

    double coronalOffset;
    double sagittalOffset;




};

} // namespace loco

#endif /* LOCO_COMOVERSUPPORTPOLYGONCONTROL_HPP_ */
