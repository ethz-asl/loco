/*!
* @file     GaitPatternAPSTest.cpp
* @author   Christian Gehring
* @date     Jun, 2013
* @version  1.0
* @ingroup
* @brief
*/

#include "loco/gait_pattern/GaitPatternAPS.hpp"
#include <gtest/gtest.h>



TEST(GaitPatternAPSTest, testAPS) {
  loco::APS aps(0.8, 0.8, 0.5, 0.5, 0.5, 0.5, 0.5);
  loco::GaitPatternAPS gaitPatternAPS;
  double dt = 0.0025;
  gaitPatternAPS.initialize(aps, dt);
  for (double t=0; t<6.0; t+=dt) {
    gaitPatternAPS.advance(dt);
  }
  gaitPatternAPS.print();
}
