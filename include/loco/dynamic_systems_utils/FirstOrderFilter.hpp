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

//! First order discrete time filter implementation.
/*!
 * This class implements the discrete time (DT) realization of a first order continuous time (CT) transfer function. In CT it is:
 *                     mu_0
 *    G(s)    =    -------------
 *                   1 + tau*s
 *
 * Using the Tustin (or bilinear) transformation (which maps stable CT poles inside the unit circle in the Z domain), being T the sample time, one has:
 *
 *                       z + 1
 *    G'(z)   =  rho * ---------
 *                       z + a
 *
 * where:
 *                     mu_0 * T
 *    rho     =     -------------
 *                     T + 2*tau
 *
 *
 *                    T - 2*tau
 *    a       =     -------------
 *                    T + 2*tau
 *
 *  In the discrete time domain, one has:
 *
 *    y(k)    =   rho*u(k) + rho*u(k-1) - a*y(k-1)
 *
 */
  class FirstOrderFilter: public FirstOrderSystemBase {
   public:

    FirstOrderFilter();
    ~FirstOrderFilter();

    /*! Initialize the filter
     * @param[in] y_0 The initial condition y(0)
     * @param[in] continuousTimeConstant The CT time constant tau (see class description)
     * @param[in] gain The CT transfer function gain mu_0 (see class description)
     */
    virtual void initialize(double y_0, double continuousTimeConstant, double gain);

    /*! Evaluate the difference equation that models the DT filter
     * @param[in] dt The current sampling time T
     * @param[in] u_k The input at time t(kT)
     * @return The output signal y[(k+1)T]
     */
    virtual double advance(double dt, double u_k);

    /*! Set the CT time constant tau
     * @param[in] continuousTimeConstant The CT time constant tau (see class description)
     */
    virtual void setContinuousTimeConstant(double continuousTimeConstant);

    /*! Set the CT gain mu_0
     * @param[in] gain The CT transfer function gain mu_0 (see class description)
     */
    virtual void setContinuousTimeGain(double continuousTimeGain);

   protected:

    double y_k_hold_;
    double u_k_hold_;
    double continuousTimeConstant_;
    double gain_;

  };

} /* namespace loco */

#endif /* LOCO_FIRSTORDERFILTER_HPP_ */
