/*!
* @file     BaseControlDynamicGait.hpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#ifndef LOCO_BASECONTROLDYNAMICGAIT_HPP_
#define LOCO_BASECONTROLDYNAMICGAIT_HPP_

#include "loco/torso_control/TorsoControlGaitContainer.hpp"
#include "loco/com_over_support_polygon/CoMOverSupportPolygonControlDynamicGait.hpp"

#include "loco/common/LegGroup.hpp"
#include "loco/common/TorsoBase.hpp"

#include "loco/common/TerrainModelBase.hpp"


namespace loco {

class TorsoControlDynamicGait: public TorsoControlGaitContainer {
 public:
  TorsoControlDynamicGait(LegGroup* legs, TorsoBase* torso, loco::TerrainModelBase* terrain);
  virtual ~TorsoControlDynamicGait();

  virtual bool initialize(double dt);
  virtual bool advance(double dt);

  CoMOverSupportPolygonControlDynamicGait* getCoMControl();
  const CoMOverSupportPolygonControlDynamicGait& getCoMControl() const;
  virtual bool loadParameters(const TiXmlHandle& handle);


  /*! Computes an interpolated version of the two controllers passed in as parameters.
   *  if t is 0, the current setting is set to controller1, 1 -> controller2, and values in between
   *  correspond to interpolated parameter set.
   * @param torsoController1
   * @param torsoController2
   * @param t interpolation parameter
   * @returns true if successful
   */
  virtual bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t);

  virtual CoMOverSupportPolygonControlDynamicGait* getCoMOverSupportPolygonControl();

 protected:
  LegGroup* legs_;
  TorsoBase* torso_;
  loco::TerrainModelBase* terrain_;
  CoMOverSupportPolygonControlDynamicGait* comControl_;

};

} /* namespace loco */

#endif /* LOCO_BASECONTROLDYNAMICGAIT_HPP_ */
