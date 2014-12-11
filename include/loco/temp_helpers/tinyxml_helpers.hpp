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
 * tinyxml_helpers.hpp
 *
 *  Created on: Sep 3, 2014
 *      Author: gech
 */

#ifndef LOCO_TINYXML_HELPERS_HPP_
#define LOCO_TINYXML_HELPERS_HPP_

#include <tinyxml.h>
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>

namespace loco {

template<enum kindr::phys_quant::PhysicalType PhysicalType_>
bool writeToXml(TiXmlHandle& handle, const kindr::vectors::eigen_impl::Vector<PhysicalType_, double, 3>& vector3) {
  TiXmlElement* element = handle.ToElement();
  element->SetDoubleAttribute("x", vector3.x());
  element->SetDoubleAttribute("y", vector3.y());
  element->SetDoubleAttribute("z", vector3.z());
  return true;

//  TiXmlElement* parent = handle.ToElement();
//  TiXmlElement* child;
//  child = handle.FirstChild("x").ToElement();
//  if(!child) {
//   child = new TiXmlElement("x");
//   parent->LinkEndChild(child);
//  }
//  child->SetDoubleAttribute("x", vector3.x());
//
//  child = handle.FirstChild("y").ToElement();
//  if(!child) {
//   child = new TiXmlElement("y");
//   parent->LinkEndChild(child);
//  }
//  child->SetDoubleAttribute("y", vector3.y());
//
//  child = handle.FirstChild("y").ToElement();
//  if(!child) {
//   child = new TiXmlElement("y");
//   parent->LinkEndChild(child);
//  }
//  child->SetDoubleAttribute("y", vector3.y());
//
//  child = handle.FirstChild("z").ToElement();
//  if(!child) {
//   child = new TiXmlElement("z");
//   parent->LinkEndChild(child);
//  }
//  child->SetDoubleAttribute("z", vector3.z());
}

template<enum kindr::phys_quant::PhysicalType PhysicalType_>
bool readFromXml(const TiXmlHandle& handle, kindr::vectors::eigen_impl::Vector<PhysicalType_, double, 3>& vector3) {
  TiXmlElement* element = handle.ToElement();
  if (!element) {
    return false;
  }
  bool result = true;
  if(element->QueryDoubleAttribute("x", &vector3.x()) != TIXML_SUCCESS) {
    result = false;
  }
  if(element->QueryDoubleAttribute("y", &vector3.y()) != TIXML_SUCCESS) {
    result = false;
  }
  if(element->QueryDoubleAttribute("z", &vector3.z()) != TIXML_SUCCESS) {
    result = false;
  }
  return result;
}

} // namespace loco

#endif /* TINYXML_HELPERS_HPP_ */
