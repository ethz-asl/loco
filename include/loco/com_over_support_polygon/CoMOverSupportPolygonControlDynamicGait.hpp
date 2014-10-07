/*!
* @file 	  CoMOverSupportPolygonControl.hpp
* @author 	Christian Gehring, Stelian Coros, Nina Sauthoff
* @date		  Jul 17, 2012
* @version 	1.0
* @ingroup
* @brief
*/

#ifndef LOCO_COMOVERSUPPORTPOLYGONCONTROLDYNAMICGAIT_HPP_
#define LOCO_COMOVERSUPPORTPOLYGONCONTROLDYNAMICGAIT_HPP_

#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlBase.hpp"

namespace loco {

//! Support Polygon Task
/*! To maintain balance, we want to keep the projected CoM within the support polygon.
 * This class computes the error vector from the center of all feet to the desired weighted location of the center of all feet in world coordinates.
 * The error vector can then be used by a virtual force controller.
 */
class CoMOverSupportPolygonControlDynamicGait: public CoMOverSupportPolygonControlBase {
  public:
    //! Constructor
    CoMOverSupportPolygonControlDynamicGait(LegGroup* legs);

    //! Destructor
    virtual ~CoMOverSupportPolygonControlDynamicGait();

    /*! Gets the error vector from the center of all feet to the desired weighted location of the center of all feet in world coordinates
     * @param legs	references to the legs
     * @return error vector expressed in world frame
     */
    virtual const Position& getDesiredWorldToCoMPositionInWorldFrame() const;

    virtual void advance(double dt);

protected:

};

} // namespace loco

#endif /* LOCO_COMOVERSUPPORTPOLYGONCONTROLDYNAMICGAIT_HPP_ */
