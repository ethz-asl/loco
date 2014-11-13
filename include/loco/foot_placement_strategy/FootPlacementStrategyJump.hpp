/*!
 * @file   FootPlacementStrategyJump.hpp
 * @author wko
 * @date   Jun 6, 2014
 * @version  1.0
 * @ingroup  robotTask
 * @brief
 */

#ifndef LOCO_FOOTPLACEMENTSTRATEGYJUMP_HPP_
#define LOCO_FOOTPLACEMENTSTRATEGYJUMP_HPP_

#include "loco/common/TypeDefs.hpp"
#include "loco/common/TorsoBase.hpp"
#include "loco/common/LegGroup.hpp"

#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"

#include "tinyxml.h"
#include <Eigen/Core>

#include "loco/temp_helpers/Trajectory.hpp"

#include "kindr/rotations/RotationEigen.hpp"
#include "robotUtils/loggers/Logger.hpp"
#include "loco/common/TerrainModelBase.hpp"

#include "robotUtils/function_approximators/polyharmonicSplines/BoundedRBF1D.hpp"

namespace loco {

/*!
 * @ingroup robotTask
 */
class FootPlacementStrategyJump : public FootPlacementStrategyBase {
 public:

  FootPlacementStrategyJump(LegGroup* legs, TorsoBase* torso,
                            loco::TerrainModelBase* terrain);
  virtual ~FootPlacementStrategyJump();

  virtual bool loadParameters(const TiXmlHandle& handle);
  virtual bool initialize(double dt);
  virtual bool advance(double dt);

 protected:
  //! Reference to the legs
  LegGroup* legs_;
  //! Reference to the torso
  TorsoBase* torso_;
  //! Reference to the terrain
  loco::TerrainModelBase* terrain_;

 protected:
  LegBase::JointPositions leftForeInitJointPositions_;



};

}  // namespace loco
#endif /* LOCO_FOOTPLACEMENTSTRATEGYJUMP_HPP_ */
