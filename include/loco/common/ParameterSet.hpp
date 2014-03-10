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

//! Parameter set
/*! Loads parameters from an XML file
 *
 */
class ParameterSet {
 public:
  //! Constructor
  ParameterSet();

  //! Destructor
  virtual ~ParameterSet();

  /*! @returns true if document is loaded
   */
  bool isDocumentLoaded();

  /*! Loads the document
   * @param filename  path and file name of the XML file
   * @returns true if document could be loaded
   */
  bool loadXmlDocument(const std::string& filename);

  /*! Gets the handle to access the parameters from the XML file
   *
   * @return  handle to the XML document
   */
  TiXmlHandle& getHandle();


 protected:
  //! true if xml document is loaded
  bool isDocumentLoaded_;

  //! XML document (see xmlDocumentHandle_)
  TiXmlDocument xmlDocument_;

  //! document handle to access the parameters
  TiXmlHandle xmlDocumentHandle_;

  //! path to parameter file
  std::string parameterFile_;
};

} /* namespace loco */

#endif /* LOCO_PARAMETERSET_HPP_ */
