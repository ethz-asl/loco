/*****************************************************************************************
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

  ParameterSet(const std::string& filename);

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
