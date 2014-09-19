/*
 * FirstOrderFilter.hpp
 *
 *  Created on: Sep 19, 2014
 *      Author: dario
 */

#ifndef LOCO_FIRSTORDERFILTER_HPP_
#define LOCO_FIRSTORDERFILTER_HPP_

#include "loco/dynamic_systems_utils/FirstOrderSystemBase.hpp"

namespace loco {

  class FirstOrderFilter: public FirstOrderSystemBase {
   public:

    FirstOrderFilter();
    ~FirstOrderFilter();

    virtual void initialize(double y_0, double continuousTimeConstant, double gain);
    virtual double advance(double dt, double u_k);

    virtual void setContinuousTimeConstant(double continuousTimeConstant);
    virtual void setContinuousTimeGain(double continuousTimeGain);

   protected:

    double y_k_hold_;
    double u_k_hold_;
    double continuousTimeConstant_;
    double gain_;

  };

} /* namespace loco */

#endif /* LOCO_FIRSTORDERFILTER_HPP_ */
