/*
 * ParameterSet.cpp
 *
 *  Created on: Mar 6, 2014
 *      Author: gech
 */

#include "loco/common/ParameterSet.hpp"

namespace loco {

ParameterSet::ParameterSet() :
    isDocumentLoaded_(false),
    xmlDocument_(),
    xmlDocumentHandle_(&xmlDocument_)
{


}

ParameterSet::~ParameterSet() {

}

bool ParameterSet::isDocumentLoaded() {
  return isDocumentLoaded_;
}

bool ParameterSet::loadXmlDocument(const std::string& filename) {
  parameterFile_ = filename;
  isDocumentLoaded_ = xmlDocument_.LoadFile(filename);
  return isDocumentLoaded_;
}

TiXmlHandle& ParameterSet::getHandle() {
  return xmlDocumentHandle_;
}

} /* namespace loco */


