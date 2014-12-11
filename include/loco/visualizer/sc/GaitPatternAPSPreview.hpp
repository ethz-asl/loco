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
#ifndef LOCO_GAITPATTERNAPSPREVIEW_HPP_
#define LOCO_GAITPATTERNAPSPREVIEW_HPP_

#include <AppGUI/SubGLWindow.h>
#include "loco/temp_helpers/Trajectory.hpp"
#include "loco/gait_pattern/GaitPatternAPS.hpp"


namespace loco {

class GaitPatternAPS;

class GaitPatternAPSPreview : public SubGLWindow {
public:
	//this is the gait we will be using for the preview...
	GaitPatternAPS* gp;
	//this is how many cycles we will be drawing...
	int nrCycles;

	double cursorPosition;
private:
	double maxXCoord;
	double maxYCoord;
	double labelsStart;
	double boxStart;
	double boxLength;
public:
	GaitPatternAPSPreview(int posX, int posY, int sizeX, int sizeY);
	virtual ~GaitPatternAPSPreview(void);


	virtual void draw();

	double getTrajMin(Trajectory1D traj);
	double getTrajMax(Trajectory1D traj);
	double getScaledY(double ymin, double ymax, double yval);
};

} // namespace loco

#endif /* LOCO_GAITPATTERNAPSPREVIEW_HPP_ */
