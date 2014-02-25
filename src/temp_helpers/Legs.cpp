#include "loco/temp_helpers/Legs.hpp"

Legs operator++(Legs& l) { return l = (Legs)(std::underlying_type<Legs>::type(l) + 1); }
Legs operator*(Legs l){ return l; }
Legs begin(Legs l){ return Legs::LEFT_FRONT; }
Legs end(Legs l){ return (Legs)((int)Legs::RIGHT_HIND + 1); }

std::map<Legs, std::string> legNames = boost::assign::map_list_of
    (Legs::LEFT_FRONT,  "left front")
    (Legs::RIGHT_FRONT, "right front")
    (Legs::LEFT_HIND,   "left hind")
    (Legs::RIGHT_HIND,  "right hind");

std::map<Legs, std::string> legNamesShort = boost::assign::map_list_of
    (Legs::LEFT_FRONT,  "LF")
    (Legs::RIGHT_FRONT, "RF")
    (Legs::LEFT_HIND,   "LH")
    (Legs::RIGHT_HIND,  "RH");

std::map<Legs, int> legIndeces = boost::assign::map_list_of
    (Legs::LEFT_FRONT,  0)
    (Legs::RIGHT_FRONT, 1)
    (Legs::LEFT_HIND,   2)
    (Legs::RIGHT_HIND,  3);
