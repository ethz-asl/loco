/*
 * VisualizerBase.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: gech
 */

#include "loco/visualizer/VisualizerBase.hpp"
#include "AppGUI/Globals.h"
namespace loco {

VisualizerBase::VisualizerBase():
    desiredFrameRate_(Globals::desiredFrameRate) {


}

VisualizerBase::~VisualizerBase() {

}

} /* namespace loco */
