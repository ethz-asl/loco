/*
 * FirstOrderSystemBase.hpp
 *
 *  Created on: Sep 19, 2014
 *      Author: dario
 */

#ifndef LOCO_FIRSTORDERSYSTEMBASE_HPP_
#define LOCO_FIRSTORDERSYSTEMBASE_HPP_


namespace loco {

  class FirstOrderSystemBase {
   public:

    FirstOrderSystemBase();
    virtual ~FirstOrderSystemBase();

    virtual double advance(double dt, double u_k) = 0;

  };

} /* namespace loco */


#endif /* LOCO_FIRSTORDERSYSTEMBASE_HPP_ */
