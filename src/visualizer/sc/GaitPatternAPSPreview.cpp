#include "loco/visualizer/sc/GaitPatternAPSPreview.hpp"

#include <include/GLheaders.h>
#include <GLUtils/GLUtils.h>
#include <stdlib.h>
#include <math.h>
#include <AppGUI/Globals.h>

namespace loco {




GaitPatternAPSPreview::GaitPatternAPSPreview(int posX, int posY, int sizeX, int sizeY) : SubGLWindow(posX, posY, sizeX, sizeY){
  gp = NULL;
  nrCycles = 1;


  cursorPosition = 0.0;//0.3;

  maxXCoord = (double) sizeX / sizeY;
  maxYCoord = 1.0;
  labelsStart = 0.05;
  boxStart = labelsStart + 0.4;
  boxLength = (maxXCoord - 0.05 - boxStart) / nrCycles;
}

GaitPatternAPSPreview::~GaitPatternAPSPreview(void){
}

double GaitPatternAPSPreview::getTrajMin(Trajectory1D traj)
{
  double ret = std::numeric_limits<double>::infinity();
  for(int i =0; i < traj.getKnotCount(); ++i)
  {
    if(traj.getKnotValue(i) < ret)
      ret = traj.getKnotValue(i);
  }
  return ret;
}
double GaitPatternAPSPreview::getTrajMax(Trajectory1D traj)
{
  double ret = -std::numeric_limits<double>::infinity();
  for(int i =0; i < traj.getKnotCount(); ++i)
  {
    if(traj.getKnotValue(i) > ret)
      ret = traj.getKnotValue(i);
  }
  return ret;
}

//scale it to [0,1], unless max/min are the same, then just put everything halfway.
double GaitPatternAPSPreview::getScaledY(double ymin, double ymax, double yval){
  if((ymax - ymin) < .01)
    return .5;
  return (yval - ymin)/(ymax - ymin);
}

void GaitPatternAPSPreview::draw(){
  if (gp == NULL)
    return;

  if (!gp->isInitialized())
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
//  for (uint i=0;i<gp->footFallPatterns.size();i++){
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
//  for (uint j = 0;j<gp->footFallPatterns.size(); j++){


  const double timeAPSStart =  gp->getCurrentAPS(0)->getTimeAPSStart();
  const double timeAPSEnd =  gp->getCurrentAPS(0)->getTimeAPSEnd();

  std::list<std::pair<double, double> > invervals[4];
  for (int iLeg=0; iLeg<4;iLeg++) {
    for (GaitPatternAPS::APSIterator it=gp->getIteratorBegin(); it!=gp->getIteratorEnd();it++) {
      if (it->getTimeFootTouchDown(iLeg) >= timeAPSStart && it->getTimeFootTouchDown(iLeg)<=timeAPSEnd) {
        if (it->getTimeFootLiftOff(iLeg) > timeAPSEnd) {
          invervals[iLeg].push_back(std::pair<double,double>(it->getTimeFootTouchDown(iLeg), timeAPSEnd));
        } else {
          invervals[iLeg].push_back(std::pair<double,double>(it->getTimeFootTouchDown(iLeg), it->getTimeFootLiftOff(iLeg)));
        }
      } else if (it->getTimeFootLiftOff(iLeg) >= timeAPSStart && it->getTimeFootLiftOff(iLeg)<=timeAPSEnd) {
        if (it->getTimeFootTouchDown(iLeg) < timeAPSStart) {
          invervals[iLeg].push_back(std::pair<double,double>(timeAPSStart, it->getTimeFootLiftOff(iLeg)));
        }
      }
      if (it->getTimeFootTouchDown(iLeg) > timeAPSEnd && it->getTimeFootLiftOff(iLeg) > timeAPSEnd) {
        break;
      }
    }
  }
  const double time = gp->getTime();
  for (uint j=0, iLeg=numFeet-1;j<numFeet;j++,iLeg--){

      for(std::list<std::pair<double, double> >::iterator it=invervals[iLeg].begin(); it!=invervals[iLeg].end(); it++) {



        const double intervalStart =  (it->first-timeAPSStart)/(timeAPSEnd-timeAPSStart);
        const double intervalEnd =  (it->second-timeAPSStart)/(timeAPSEnd-timeAPSStart);
        const double invervalCurrent =  (time-timeAPSStart)/(timeAPSEnd-timeAPSStart);
  //      printf("Leg %d: Cycle: %d: LO: %f TD: %f IS: %f IE: %f\n", f, i, aps->timeFootLiftOff_, aps->timeFootTouchDown_, intervalStart, intervalEnd);
//        boundToRange(&intervalStart, 0, nrCycles);
//        boundToRange(&intervalEnd, 0, nrCycles);
        double colour = gp->getStancePhase(iLeg);
  //      if (colour > 0)
  //        glColor3d(0.0,1.3-colour,0.0);
  //      else
  //        glColor3d(0.0,0.0,0.0);
  //
//        glColor3d(0.0,0.0,0.0);
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

        if (gp->shouldBeGrounded(iLeg)) {
          if (intervalStart <= invervalCurrent && intervalEnd >= invervalCurrent) {
//            glColor3d(0.0,0.5,0.0);
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


//  const double time = gp->getTime();
//  APS* aps;
//
//  for (uint j=0, f=gp->footFallPatterns.size()-1;j<gp->footFallPatterns.size();j++,f--){
//    int iLeg = f;
//
//
//    GaitPatternAPS::APSIterator currentAPS = gp->currentAPS[iLeg);
//
//
//    for (int i=-1;i<=nrCycles;i++){
////      if (i == -2) {
////
////
////      }else if (i == -1) {              {
////        if ( gp->getCurrentAPS(0))
//
////        if (gp->getCurrentAPS(f) == gp->getPreviousAPS(0))
////          aps = gp->getCurrentAPS(f);
////        else if (gp->getPreviousAPS(f) == gp->getPreviousAPS(0)) {
////          aps = gp->getPreviousAPS(f);
////        } else if (gp->getNextAPS(f) == gp->getPreviousAPS(0)) {
////          aps = gp->getNextAPS(f);
////        } else if (gp->getNextNextAPS(f) == gp->getPreviousAPS(0)) {
////          aps = gp->getNextNextAPS(f);
////        } else if (gp->getPreviousPreviousAPS(f) == gp->getPreviousAPS(0)) {
////          aps = gp->getPreviousPreviousAPS(f);
////        } else if (gp->getNextNextAPS(f) == gp->getCurrentAPS(0)) {
////          aps = gp->getNextNextAPS(f);
////          printf("%d next next\n", f);
////        } else {
////          throwError("GaitPatternAPSPreview: wrong index!\n");
////        }
////        glColor3d(0.0,0.5,0.0);
////      } else if(i == 0) {
////        if (gp->getCurrentAPS(f) == gp->getCurrentAPS(0))
////          aps = gp->getCurrentAPS(f);
////        else if (gp->getPreviousAPS(f) == gp->getCurrentAPS(0)) {
////          aps = gp->getPreviousAPS(f);
////        } else if (gp->getNextAPS(f) == gp->getCurrentAPS(0)) {
////          aps = gp->getNextAPS(f);
////        } else if (gp->getNextNextAPS(f) == gp->getCurrentAPS(0)) {
////          aps = gp->getNextNextAPS(f);
////        } else if (gp->getPreviousPreviousAPS(f) == gp->getCurrentAPS(0)) {
////          aps = gp->getPreviousPreviousAPS(f);
////        }
//////        glColor3d(0.0,0.0,0.0);
////      } else if(i == 1) {
////        if (gp->getCurrentAPS(f) == gp->getNextAPS(0))
////          aps = gp->getCurrentAPS(f);
////        else if (gp->getPreviousAPS(f) == gp->getNextAPS(0)) {
////          aps = gp->getPreviousAPS(f);
////        } else if (gp->getNextAPS(f) == gp->getNextAPS(0)) {
////          aps = gp->getNextAPS(f);
////        }
//////        glColor3d(0.5,0.0,0.0);
////      } else {
////        throwError("GaitPatternAPSPreview: wrong index!\n");
////      }
//
////      aps = gp->getCurrentAPS(f);
//
////      switch (iLeg) {
////      case 0:
////        if (i==0) {
////          aps = gp->getCurrentAPS(0);
////        } else if (i == -1) {
////          aps = gp->getPreviousAPS(0);
////        } else if (i == 1) {
////          aps = gp->getNextAPS(0);
////        }
////        break;
////      default:
////        ;
////      }
//
////      if (i == -1)
////      {
////        aps = gp->getPreviousAPS(f);
////  //      glColor3d(0.0,0.5,0.0);
////      } else if(i == 0) {
////        aps = gp->getCurrentAPS(f);
////  //        glColor3d(0.0,0.0,0.0);
////      } else if(i == 1) {
////        aps = gp->getNextAPS(f);
////  //        glColor3d(0.5,0.0,0.0);
////      } else {
////        throwError("GaitPatternAPSPreview: wrong index!\n");
////      }
//
//
//      double intervalStart = i + (aps->getTimeFootTouchDown(f]-aps->startTime_)/aps->cycleDuration_; //gp->footFallPatterns[f].footLiftOff;
//      double intervalEnd = i + (aps->getTimeFootLiftOff(f]-aps->startTime_)/aps->cycleDuration_;//gp->footFallPatterns[f].footStrike;
////      printf("Leg %d: Cycle: %d: LO: %f TD: %f IS: %f IE: %f\n", f, i, aps->timeFootLiftOff_, aps->timeFootTouchDown_, intervalStart, intervalEnd);
//      boundToRange(&intervalStart, 0, nrCycles);
//      boundToRange(&intervalEnd, 0, nrCycles);
//      double colour = gp->getStancePhaseForLeg(gp->footFallPatterns[f].leg, cursorPosition);
////      if (colour > 0)
////        glColor3d(0.0,1.3-colour,0.0);
////      else
////        glColor3d(0.0,0.0,0.0);
////
//      glColor3d(0.0,0.0,0.0);
//      if (gp->footFallPatterns[f].leg->isAndShouldBeGrounded())
//        glColor3d(0.0,0.5,0.0);
//
//      if (intervalStart != intervalEnd){
//        //now draw the interval - in the ith column, jth row...
//        glVertex2d(boxStart + intervalStart*boxLength, 0.075 + j*boxWidth);
//        glVertex2d(boxStart + intervalEnd*boxLength, 0.075 + j*boxWidth);
//        glVertex2d(boxStart + intervalEnd*boxLength, 0.075 + (j+1)*boxWidth - 0.05);
//        glVertex2d(boxStart + intervalStart*boxLength, 0.075 + (j+1)*boxWidth - 0.05);
//      }
//    }
//  }
//  glEnd();

  //finally, draw the cursor
//  if (cursorPosition < 0) cursorPosition += 1;
//  if (cursorPosition > nrCycles) cursorPosition -= nrCycles;
    cursorPosition =  gp->getCurrentAPS(0)->phase_;

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




} // namespace loco
