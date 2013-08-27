#pragma once

/*!
 * TODO move this in some form to robot commons
 */

#include <boost/assign.hpp>
#include <map>
#include <string>

enum class Legs : int {
  LEFT_FRONT = 0,
  RIGHT_FRONT,
  LEFT_HIND,
  RIGHT_HIND
};

Legs operator++(Legs& l);
Legs operator*(Legs l);
Legs begin(Legs l);
Legs end(Legs l);

extern std::map<Legs, std::string> legNames;

extern std::map<Legs, std::string> legNamesShort;

extern std::map<Legs, int> legIndeces;
