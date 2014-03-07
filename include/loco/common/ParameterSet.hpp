/*
 * ParameterSet.hpp
 *
 *  Created on: Mar 6, 2014
 *      Author: gech
 */

#ifndef LOCO_PARAMETERSET_HPP_
#define LOCO_PARAMETERSET_HPP_

#include <string>
#include "tinyxml.h"
namespace loco {

class ParameterSet {
 public:
  ParameterSet();
  virtual ~ParameterSet();
  bool isDocumentLoaded();
  bool loadXmlDocument(const std::string& filename);
  TiXmlHandle& getHandle();
 protected:
  bool isDocumentLoaded_;

  TiXmlDocument xmlDocument_;
  TiXmlHandle xmlDocumentHandle_;
  std::string parameterFile_;
};

} /* namespace loco */

#endif /* LOCO_PARAMETERSET_HPP_ */
