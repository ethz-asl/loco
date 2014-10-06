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
