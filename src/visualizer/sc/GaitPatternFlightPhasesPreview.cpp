/*
 * GaitPatternFlightPhasesPreview.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: gech
 */

#include "loco/visualizer/sc/GaitPatternFlightPhasesPreview.hpp"

#include <include/GLheaders.h>
#include <GLUtils/GLUtils.h>
#include <stdlib.h>
#include <math.h>
#include <AppGUI/Globals.h>

namespace loco {

GaitPatternFlightPhasesPreview::GaitPatternFlightPhasesPreview(int posX, int posY, int sizeX, int sizeY) :
    SubGLWindow(posX, posY, sizeX, sizeY),
    gaitPattern_(nullptr),
    cursorPosition(0.0),
    nrCycles(1)
{


  maxXCoord = (double) sizeX / sizeY;
  maxYCoord = 1.0;
  labelsStart = 0.05;
  boxStart = labelsStart + 0.4;
  boxLength = (maxXCoord - 0.05 - boxStart) / nrCycles;
}

GaitPatternFlightPhasesPreview::~GaitPatternFlightPhasesPreview() {

}

void GaitPatternFlightPhasesPreview::draw() {
  if (gaitPattern_ == nullptr)
    return;

  if (!gaitPattern_->isInitialized())
    return;

  SubGLWindow::preDraw();

  //clear a box for this...
  glColor3d(1, 1, 1);
  glBegin(GL_QUADS);
    glVertex2d(0, 0);
    glVertex2d(0, 1);
    glVertex2d(maxXCoord, 1);
    glVertex2d(maxXCoord, 0);
  glEnd();


  //draw the vertical lines that delineate the different cycles
  glColor3d(0.7, 0.7, 0.7);
  glBegin(GL_LINES);
  for (int i=0;i<=nrCycles;i++){
    glVertex2d(boxStart + i*boxLength, 0.05);
    glVertex2d(boxStart + i*boxLength, 0.95);
  }
  glEnd();

  //draw the horizontal lines that delineate the different legs used...
  glBegin(GL_LINES);
  const int numFeet = 4;
  int numSections = numFeet;

  double boxWidth = (maxYCoord - 0.1) / numSections;
  for (int i=0;i<=numSections;i++){
    glVertex2d(labelsStart, 0.05 + i*boxWidth);
    glVertex2d(boxStart + nrCycles * boxLength, 0.05 + i*boxWidth);
  }
  glEnd();

  glColor3d(0.0,0.0,0.0);

  //now draw the labels...
//  for (uint i=0;i<gaitPattern_->footFallPatterns.size();i++){
  for (uint i=0, j=numFeet-1;i<numFeet;i++,j--){
    glRasterPos2d(labelsStart, 0.05 + i*boxWidth + boxWidth/3.0);
    std::string name;
    if (j==0) name = "LF";
    if (j==1) name = "RF";
    if (j==2) name = "LH";
    if (j==3) name = "RH";

    glargeprintf("%s\n",  name.c_str());
  }

  glBegin(GL_QUADS);
  //finally, draw all the regions where the legs are supposed to be in swing mode
//  for (uint j = 0;j<gaitPattern_->footFallPatterns.size(); j++){


//  const double timeAPSStart =  gaitPattern_->getCurrentAPS(0)->getTimeAPSStart();
//  const double timeAPSEnd =  gaitPattern_->getCurrentAPS(0)->getTimeAPSEnd();
//
//  std::list<std::pair<double, double> > invervals[4];
//  for (int iLeg=0; iLeg<4;iLeg++) {
//    for (GaitPatternAPS::APSIterator it=gaitPattern_->getIteratorBegin(); it!=gaitPattern_->getIteratorEnd();it++) {
//      if (it->getTimeFootTouchDown(iLeg) >= timeAPSStart && it->getTimeFootTouchDown(iLeg)<=timeAPSEnd) {
//        if (it->getTimeFootLiftOff(iLeg) > timeAPSEnd) {
//          invervals[iLeg].push_back(std::pair<double,double>(it->getTimeFootTouchDown(iLeg), timeAPSEnd));
//        } else {
//          invervals[iLeg].push_back(std::pair<double,double>(it->getTimeFootTouchDown(iLeg), it->getTimeFootLiftOff(iLeg)));
//        }
//      } else if (it->getTimeFootLiftOff(iLeg) >= timeAPSStart && it->getTimeFootLiftOff(iLeg)<=timeAPSEnd) {
//        if (it->getTimeFootTouchDown(iLeg) < timeAPSStart) {
//          invervals[iLeg].push_back(std::pair<double,double>(timeAPSStart, it->getTimeFootLiftOff(iLeg)));
//        }
//      }
//      if (it->getTimeFootTouchDown(iLeg) > timeAPSEnd && it->getTimeFootLiftOff(iLeg) > timeAPSEnd) {
//        break;
//      }
//    }
//  }
//  const double time = gaitPattern_->getTime();
//  for (uint j=0, iLeg=numFeet-1;j<numFeet;j++,iLeg--){
//
//      for(std::list<std::pair<double, double> >::iterator it=invervals[iLeg].begin(); it!=invervals[iLeg].end(); it++) {
//
//
//
//        const double intervalStart =  (it->first-timeAPSStart)/(timeAPSEnd-timeAPSStart);
//        const double intervalEnd =  (it->second-timeAPSStart)/(timeAPSEnd-timeAPSStart);
//        const double invervalCurrent =  (time-timeAPSStart)/(timeAPSEnd-timeAPSStart);
//  //      printf("Leg %d: Cycle: %d: LO: %f TD: %f IS: %f IE: %f\n", f, i, aps->timeFootLiftOff_, aps->timeFootTouchDown_, intervalStart, intervalEnd);
////        boundToRange(&intervalStart, 0, nrCycles);
////        boundToRange(&intervalEnd, 0, nrCycles);
//        double colour = gaitPattern_->getStancePhase(iLeg);
//  //      if (colour > 0)
//  //        glColor3d(0.0,1.3-colour,0.0);
//  //      else
//  //        glColor3d(0.0,0.0,0.0);
//  //
////        glColor3d(0.0,0.0,0.0);
//        switch (iLeg) {
//        case 0:
//          glColor3d(247.0/255.0,204.0/255.0,212.0/255.0);
//          break;
//        case 1:
//          glColor3d(250.0/255.0,237.0/255.0,204.0/255.0);
//          break;
//        case 2:
//          glColor3d(204.0/255.0,223.0/255.0,212.0/255.0);
//          break;
//        case 3:
//          glColor3d(214.0/255.0,240.0/255.0,214.0/255.0);
//          break;
//        default:
//          glColor3d(0.0,0.0,0.0);
//        }
//
//        if (gaitPattern_->shouldBeGrounded(iLeg)) {
//          if (intervalStart <= invervalCurrent && intervalEnd >= invervalCurrent) {
////            glColor3d(0.0,0.5,0.0);
//            switch (iLeg) {
//              case 0:
//                glColor3d(214.0/255.0,1.0/255.0,39.0/255.0);
//                break;
//              case 1:
//                glColor3d(231.0/255.0,165.0/255.0,0.0/255.0);
//                break;
//              case 2:
//                glColor3d(0.0/255.0,96.0/255.0,40.0/255.0);
//                break;
//              case 3:
//                glColor3d(49.0/255.0,180.0/255.0,48.0/255.0);
//                break;
//              default:
//                glColor3d(0.0,0.0,0.0);
//              }
//          }
//        }
//
//        if (intervalStart != intervalEnd){
//          //now draw the interval - in the ith column, jth row...
//          glVertex2d(boxStart + intervalStart*boxLength, 0.075 + j*boxWidth);
//          glVertex2d(boxStart + intervalEnd*boxLength, 0.075 + j*boxWidth);
//          glVertex2d(boxStart + intervalEnd*boxLength, 0.075 + (j+1)*boxWidth - 0.05);
//          glVertex2d(boxStart + intervalStart*boxLength, 0.075 + (j+1)*boxWidth - 0.05);
//        }
//      }
//    }
//    glEnd();



  for (uint j=0, f=numFeet-1;j<numFeet;j++,f--){
    int iLeg = f;




    for (int i=-1;i<=nrCycles;i++){


      double intervalStart = i + gaitPattern_->getFootLiftOffPhase(iLeg); //gaitPattern_->footFallPatterns[f].footLiftOff;
      double intervalEnd = i + gaitPattern_->getFootTouchDownPhase(iLeg);//gaitPattern_->footFallPatterns[f].footStrike;
//      printf("Leg %d: Cycle: %d: LO: %f TD: %f IS: %f IE: %f\n", f, i, aps->timeFootLiftOff_, aps->timeFootTouchDown_, intervalStart, intervalEnd);
      boundToRange(&intervalStart, 0, nrCycles);
      boundToRange(&intervalEnd, 0, nrCycles);
      double colour = gaitPattern_->getStancePhaseForLeg(iLeg);
//      if (colour > 0)
//        glColor3d(0.0,1.3-colour,0.0);
//      else
//        glColor3d(0.0,0.0,0.0);
//
//      glColor3d(0.0,0.0,0.0);
//      if (gaitPattern_->shouldBeLegGrounded(iLeg))
//        glColor3d(0.0,0.5,0.0);
//      glColor3d(1.0,1.0,1.0);
      glColor3d(0.0,0.0,0.0);


      if (intervalStart != intervalEnd){
        //now draw the interval - in the ith column, jth row...
        glVertex2d(boxStart + intervalStart*boxLength, 0.075 + j*boxWidth);
        glVertex2d(boxStart + intervalEnd*boxLength, 0.075 + j*boxWidth);
        glVertex2d(boxStart + intervalEnd*boxLength, 0.075 + (j+1)*boxWidth - 0.05);
        glVertex2d(boxStart + intervalStart*boxLength, 0.075 + (j+1)*boxWidth - 0.05);
      }
    }
  }
  glEnd();

  //finally, draw the cursor
//  if (cursorPosition < 0) cursorPosition += 1;
//  if (cursorPosition > nrCycles) cursorPosition -= nrCycles;
    cursorPosition =  gaitPattern_->getStridePhase();

//  glColor3d(1.0, 0, 0);
  glColor3d(0.0, 0, 1.0);
  glLineWidth(2.0);
  glBegin(GL_LINES);
    glVertex2d(boxStart + cursorPosition*boxLength, 0.05);
    glVertex2d(boxStart + cursorPosition*boxLength, 0.95);
  glEnd();
  glLineWidth(1.0);

  glBegin(GL_TRIANGLES);
    glVertex2d(boxStart + cursorPosition*boxLength, 0.05);
    glVertex2d(boxStart + cursorPosition*boxLength-0.05, 0.0);
    glVertex2d(boxStart + cursorPosition*boxLength+0.05, 0.0);

    glVertex2d(boxStart + cursorPosition*boxLength, 0.95);
    glVertex2d(boxStart + cursorPosition*boxLength-0.05, 1);
    glVertex2d(boxStart + cursorPosition*boxLength+0.05, 1);
  glEnd();
//  glColor3d(0.0, 0, 0.0);
//  glRasterPos2d(3.0,0.5);
//      gprintf("Speed: %0.2f m/s\n",  gp->getVelocity());

  //AND DONE!


  // Restore attributes
  SubGLWindow::postDraw();
}

void GaitPatternFlightPhasesPreview::setGaitPattern(GaitPatternFlightPhases* gaitPattern) {
  gaitPattern_ = gaitPattern;
}

} /* namespace loco */
