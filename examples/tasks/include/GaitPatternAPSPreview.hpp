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
