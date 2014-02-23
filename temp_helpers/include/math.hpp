/*
 * math.hpp
 *
 *  Created on: Feb 23, 2014
 *      Author: gech
 */

#ifndef LOCO_MATH_HPP_
#define LOCO_MATH_HPP_

#include <cmath>

namespace loco {

inline void boundToRange(double* v, double min, double max){
  if (*v < min) *v = min;
  if (*v > max) *v = max;
}


/*
  if v < min, this method returns 0. If v > max, it returns 1. For everything else it returns some interpolated value;
*/
inline double mapTo01Range(double v, double min, double max){
  double t = v;
  if (fabs(min - max) < 0.000000001) return 1;
  boundToRange(&t, min, max);
  t = (t-min)/(max-min);
  return t;
}

inline double mapToUniformRange(double v, double min, double max){
  double t = v;
  if (fabs(min - max) < 0.000000001) return 1;
  t = (t-min)/(max-min);
  return t;
}

inline double linearlyInterpolate(double v1, double v2, double t1, double t2, double t){
  if (v1 == v2)
    return v2;
  return (t-t1)/(t2-t1) * v2 + (t2-t)/(t2-t1) * v1;
}


} // namespace loco

#endif /* LOCO_MATH_HPP_ */
