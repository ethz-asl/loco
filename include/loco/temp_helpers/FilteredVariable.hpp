/*******************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Christian Gehring, Péter Fankhauser, C. Dario Bellicoso, Stelian Coros
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*
 * FilteredVariable.hpp
 *
 *  Created on: Mar 31, 2014
 *      Author: gech
 */

#ifndef LOCO_FILTEREDVARIABLE_HPP_
#define LOCO_FILTEREDVARIABLE_HPP_

#include "math.hpp"

namespace loco {


/**
  this is a class that can be used for second-order filtered variables
*/
class SecondOrderFilteredVariable {
public:
  double x_oldest, x_old;
  double y_oldest, y_old;

  SecondOrderFilteredVariable(){
    x_oldest = x_old = y_oldest = y_old = 0;
  }

  void update(double x_new, double (*filter)(double x_oldest, double x_old, double x_new, double y_oldest, double y_old, double filtereParameter) = NULL, double filterParameter = 0){
    double y_new = x_new;
    if (filter)
      y_new = filter(x_oldest, x_old, x_new, y_oldest, y_old, filterParameter);
    x_oldest = x_old;
    x_old = x_new;
    y_oldest = y_old;
    y_old = y_new;
  }

  double getCurrentValue(){
    return y_old;
  }

  void setCurrentValue(double v){
      y_old = v;
  }
};

/**
  implements a low-pass filter.
*/
template <class T> class FilteredVariable{
public:
  T value;
  double alpha;
  bool initialized;

  T& operator = (const T& other){

  }

public:
  FilteredVariable(void){
    //uninitialized, because I don't know what T is...
    initialized = false;
    alpha = 0.6;
  }

  FilteredVariable(const T& other){
    //require a copy operator...
    this->value = other;
    initialized = true;
    alpha = 0.6;
  }

  FilteredVariable(const T& other, double a){
    //require a copy operator...
    this->value = other;
    initialized = true;
    alpha = a;
  }

  void setAlpha(double a){
    alpha = a;
    boundToRange(&alpha, 0, 1);
  }

  ~FilteredVariable(void){
    //nothing to do...
  }

  void initialize(const T& other){
    //require a copy operator...
    this->value = other;
    initialized = true;
  }

  void reset(){
    initialized = false;
  }

  void update(const T& newVal){
    if (initialized == false)
      initialize(newVal);
    else
      value = newVal * alpha + value * (1-alpha);
  }

  bool isInitialized(){
    return initialized;
  }

  T val(){
    return value;
  }
};


typedef FilteredVariable<double> FilteredDouble;




}



#endif /* LOCO_FILTEREDVARIABLE_HPP_ */
