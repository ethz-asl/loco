/*
 * FirstOrderFilter.cpp
 *
 *  Created on: Sep 19, 2014
 *      Author: dario
 */

#include "loco/dynamic_systems_utils/FirstOrderFilter.hpp"
#include <iostream>


namespace loco {


  FirstOrderFilter::FirstOrderFilter():
      y_k_hold_(0.0),
      u_k_hold_(0.0),
      continuousTimeConstant_(1.0),
      gain_(1.0)
  {

  }


  FirstOrderFilter::~FirstOrderFilter() {

  }


  void FirstOrderFilter::initialize(double y_0, double continuousTimeConstant, double gain) {
    setContinuousTimeConstant(continuousTimeConstant);
    gain_ = gain;
    y_k_hold_ = y_0;
  }


  double FirstOrderFilter::advance(double dt, double u_k) {

    double rho = gain_*dt/(dt+2.0*continuousTimeConstant_);
    double a = (dt-2.0*continuousTimeConstant_)/(dt+2.0*continuousTimeConstant_);

    double y_k = rho*u_k + rho*u_k_hold_ - a*y_k_hold_;

    y_k_hold_ = y_k;
    u_k_hold_ = u_k;

    return y_k;

  }


  void FirstOrderFilter::setContinuousTimeConstant(double continuousTimeConstant) {

    if (continuousTimeConstant < 0.0) {
      std::cout << "WARNING: setting a negative time constant to continuous time first order filter" << std::endl;
    }

    continuousTimeConstant_ = continuousTimeConstant;
  }


  void FirstOrderFilter::setContinuousTimeGain(double continuousTimeGain) {
    gain_ = continuousTimeGain;
  }


} /* namespace loco */
