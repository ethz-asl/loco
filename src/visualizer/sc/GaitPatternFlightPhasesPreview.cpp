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
    cursorPosition_(0.0),
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

  const int numFeet = 4;
  int numSections = numFeet;
  double boxWidth = (maxYCoord - 0.1) / numSections;

  SubGLWindow::preDraw();



//  SubGLWindow::postDraw();
//  return;
//  for (uint i=0, j=numFeet-1;i<numFeet;i++,j--){
//    glRasterPos2d(labelsStart, 0.05 + i*boxWidth + boxWidth/3.0);
//  }


  //clear a box for this...
  glColor3d(1, 1, 1);
  glBegin(GL_QUADS);
    glVertex2d(0, 0);
    glVertex2d(0, 1);
    glVertex2d(maxXCoord, 1);
    glVertex2d(maxXCoord, 0);
  glEnd();

  // draw white box
  glEnable(GL_DEPTH_TEST);
  glColor3d(0.0,0.0,0.0);
  glBegin(GL_QUADS);
    glColor3d(1.0, 1.0, 1.0);
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

  glColor3d(0.0,0.0,0.0);

  glBegin(GL_QUADS);
   for (uint j=0, f=numFeet-1;j<numFeet;j++,f--){
     const int iLeg = f;
     for (int i=-1;i<=nrCycles;i++){
       double intervalStart = i + 0; //gaitPattern_->footFallPatterns[f].footLiftOff;
       double intervalEnd = i + 1;//gaitPattern_->footFallPatterns[f].footStrike;
       boundToRange(&intervalStart, 0, nrCycles);
       boundToRange(&intervalEnd, 0, nrCycles);
       double colour = gaitPattern_->getStancePhaseForLeg(iLeg);

           if (gaitPattern_->shouldBeLegGrounded(iLeg)) {
             switch (iLeg) {
               case 0:
                 glColor3d(214.0/255.0,1.0/255.0,39.0/255.0);
                 break;
               case 1:
                 glColor3d(231.0/255.0,165.0/255.0,0.0/255.0);
                 break;
               case 2:
                 glColor3d(0.0/255.0,96.0/255.0,40.0/255.0);
                 break;
               case 3:
                 glColor3d(49.0/255.0,180.0/255.0,48.0/255.0);
                 break;
               default:
                 glColor3d(0.0,0.0,0.0);
               }
             }
             else {
               switch (iLeg) {
               case 0:
                 glColor3d(247.0/255.0,204.0/255.0,212.0/255.0);
                 break;
               case 1:
                 glColor3d(250.0/255.0,237.0/255.0,204.0/255.0);
                 break;
               case 2:
                 glColor3d(204.0/255.0,223.0/255.0,212.0/255.0);
                 break;
               case 3:
                 glColor3d(214.0/255.0,240.0/255.0,214.0/255.0);
                 break;
               default:
                 glColor3d(0.0,0.0,0.0);
               }
             }

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




  glBegin(GL_QUADS);
  //finally, draw all the regions where the legs are supposed to be in swing mode

  for (uint j=0, f=numFeet-1;j<numFeet;j++,f--){
    int iLeg = f;
    for (int i=-1;i<=nrCycles;i++){

      double intervalStart = i + gaitPattern_->getFootLiftOffPhase(iLeg); //gaitPattern_->footFallPatterns[f].footLiftOff;
      double intervalEnd = i + gaitPattern_->getFootTouchDownPhase(iLeg);//gaitPattern_->footFallPatterns[f].footStrike;
//      printf("Leg %d: Cycle: %d: LO: %f TD: %f IS: %f IE: %f\n", f, i, aps->timeFootLiftOff_, aps->timeFootTouchDown_, intervalStart, intervalEnd);
      boundToRange(&intervalStart, 0, nrCycles);
      boundToRange(&intervalEnd, 0, nrCycles);
      double colour = gaitPattern_->getStancePhaseForLeg(iLeg);

      glColor3d(1.0,1.0,1.0);

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


//  glBegin(GL_QUADS);
//    glColor3d(0.0,0.0,0.0);
//    glColor3d(1.0, 0.1, 0.1);
//    glVertex2d(boxStart + -boxLength, -0.075 +  0*boxWidth);
//    glVertex2d(boxStart + boxLength, -0.075 + 0*boxWidth);
//    glVertex2d(boxStart + boxLength, 4.0*boxWidth);
//    glVertex2d(boxStart + -boxLength, 4.0*boxWidth);
////    glVertex2d(editorPosX, editorPosY);
////    glVertex2d(editorPosX, editorPosY+editorPosY);
////    glVertex2d(editorPosX+editorSizeX, editorPosY+editorPosY);
////    glVertex2d(editorPosX+editorSizeX, editorPosY);
//  glEnd();



  //finally, draw the cursor
    cursorPosition_ =  gaitPattern_->getStridePhase();

//  glColor3d(1.0, 0, 0);
  glColor3d(0.0, 0, 1.0);
  glLineWidth(2.0);
  glBegin(GL_LINES);
    glVertex2d(boxStart + cursorPosition_*boxLength, 0.05);
    glVertex2d(boxStart + cursorPosition_*boxLength, 0.95);
  glEnd();
  glLineWidth(1.0);

  glBegin(GL_TRIANGLES);
    glVertex2d(boxStart + cursorPosition_*boxLength, 0.05);
    glVertex2d(boxStart + cursorPosition_*boxLength-0.05, 0.0);
    glVertex2d(boxStart + cursorPosition_*boxLength+0.05, 0.0);

    glVertex2d(boxStart + cursorPosition_*boxLength, 0.95);
    glVertex2d(boxStart + cursorPosition_*boxLength-0.05, 1);
    glVertex2d(boxStart + cursorPosition_*boxLength+0.05, 1);
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
