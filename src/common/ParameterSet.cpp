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

ParameterSet::ParameterSet(const std::string& filename) :
    isDocumentLoaded_(false),
    xmlDocument_(),
    xmlDocumentHandle_(&xmlDocument_)
{
  loadXmlDocument(filename);
  if (!isDocumentLoaded_) {
    std::cout << "Could not load parameter file: " << parameterFile_  << std::endl;
  }
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


