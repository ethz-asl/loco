/*!
* @file     BaseControlBase.cpp
* @author   Christian Gehring
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/
#include "loco/torso_control/TorsoControlBase.hpp"
#include "loco/temp_helpers/math.hpp"
#include <exception>

//colored strings
const std::string black     = "\033[0;30m";
const std::string red       = "\033[0;31m";
const std::string green     = "\033[0;32m";
const std::string yellow    = "\033[0;33m";
const std::string blue      = "\033[0;34m";
const std::string magenta   = "\033[0;35m";
const std::string cyan      = "\033[0;36m";
const std::string white     = "\033[0;37m";
const std::string def       = "\033[0m";

namespace loco {


TorsoControlBase::TorsoControlBase():
    headingDistanceFromForeToHindInBaseFrame_(0.0)
{
  std::vector<double> tValues, xValues;
  const double defaultHeight = 0.42;
  desiredTorsoForeHeightAboveGroundInWorldFrameOffset_ = defaultHeight;
  desiredTorsoHindHeightAboveGroundInWorldFrameOffset_ = defaultHeight;
  desiredTorsoCoMHeightAboveGroundInControlFrameOffset_ = defaultHeight;
  tValues.push_back(0.00); xValues.push_back(0.0);
  tValues.push_back(0.25); xValues.push_back(0.0);
  tValues.push_back(0.50); xValues.push_back(0.0);
  tValues.push_back(0.75); xValues.push_back(0.0);
  tValues.push_back(1.00); xValues.push_back(0.0);
  desiredTorsoForeHeightAboveGroundInWorldFrame_.setRBFData(tValues, xValues);
  desiredTorsoHindHeightAboveGroundInWorldFrame_.setRBFData(tValues, xValues);
}


TorsoControlBase::~TorsoControlBase() {

}


//bool TorsoControlBase::setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t) {
//  return false;
//}


inline double safeACOS(double val){
  if (val<-1)
    return M_PI;
  if (val>1)
    return 0;
  return acos(val);
}


/**
  Assume that the current quaternion represents the relative orientation between two coordinate frames A and B.
  This method decomposes the current relative rotation into a twist of frame B around the axis v passed in as a
  parameter, and another more arbitrary rotation.

  AqB = AqT * TqB, where T is a frame that is obtained by rotating frame B around the axis v by the angle
  that is returned by this function.

  In the T coordinate frame, v is the same as in B, and AqT is a rotation that aligns v from A to that
  from T.

  It is assumed that vB is a unit vector!! This method returns TqB, which represents a twist about
  the axis vB.
*/
RotationQuaternion TorsoControlBase::decomposeRotation(const RotationQuaternion& AqB, const Vector& vB) {
  const Vector vA =  AqB.inverseRotate(vB).normalized();

  Vector rotAxis = (vA.cross(vB).normalized());

  if (rotAxis.norm() == 0) {
    rotAxis = Vector::UnitZ();
  }
  rotAxis *= -1.0;
  double rotAngle = -safeACOS(vA.dot(vB));
  const RotationQuaternion TqA = RotationQuaternion(AngleAxis(rotAngle, rotAxis.toImplementation()));
  return AqB*TqA; // TqB
}


RotationQuaternion TorsoControlBase::computeHeading(const RotationQuaternion& rquat, const Vector& axis) {
  return decomposeRotation(rquat.conjugated(),axis).conjugated();
}


double TorsoControlBase::getDesiredTorsoForeHeightAboveGroundInWorldFrameOffset() const {
  return desiredTorsoForeHeightAboveGroundInWorldFrameOffset_;
}


double TorsoControlBase::getDesiredTorsoHindHeightAboveGroundInWorldFrameOffset() const {
  return desiredTorsoHindHeightAboveGroundInWorldFrameOffset_;
}


void TorsoControlBase::setDesiredPositionOffsetInWorldFrame(const Position& positionTargetOffsetInWorldFrame) {
  desiredPositionOffsetInWorldFrame_ = positionTargetOffsetInWorldFrame;
}


void TorsoControlBase::setDesiredOrientationOffset(const RotationQuaternion& orientationOffset) {
  desiredOrientationOffset_ = orientationOffset;
}


bool TorsoControlBase::interpolateHeightTrajectory(rbf::PeriodicRBF1DC1& interpolatedTrajectory, const rbf::PeriodicRBF1DC1& trajectory1, const rbf::PeriodicRBF1DC1& trajectory2, double t) {

  const int nKnots = std::max<int>(trajectory1.getKnotCount(), trajectory2.getKnotCount());
  const double tMax1 = trajectory1.getKnotPosition(trajectory1.getKnotCount()-1);
  const double tMax2 = trajectory2.getKnotPosition(trajectory2.getKnotCount()-1);
  const double tMin1 = trajectory1.getKnotPosition(0);
  const double tMin2 = trajectory2.getKnotPosition(0);
  double tMax = std::max<double>(tMax1, tMax2);
  double tMin = std::min<double>(tMin1, tMin2);

  double dt = (tMax-tMin)/(nKnots-1);
  std::vector<double> tValues, xValues;
  for (int i=0; i<nKnots;i++){
    const double time = tMin + i*dt;
    double v = linearlyInterpolate(trajectory1.evaluate(time), trajectory2.evaluate(time), 0.0, 1.0, t);
    tValues.push_back(time);
    xValues.push_back(v);
//    printf("(%f,%f / %f, %f) ", time,v, legProps1->swingFootHeightTrajectory.evaluate(time), legProps2->swingFootHeightTrajectory.evaluate(time));
  }

  interpolatedTrajectory.setRBFData(tValues, xValues);
  return true;
}


bool TorsoControlBase::loadParametersTorsoConfiguration(const TiXmlHandle& hTorsoConfiguration) {

  // Check if "TorsoConfiguration" exists in parameter file
  TiXmlElement* pElem;
  pElem = hTorsoConfiguration.Element();
  if (!pElem) {
    printf("Could not find TorsoConfiguration\n");
    std::cout << magenta << "[TorsoControlBase/loadParametersTorsoConfiguration] "
              << red << "Error: "
              << blue << "could not find section 'TorsoConfiguration'."
              << def << std::endl;
    return false;
  }

  TiXmlElement* child = hTorsoConfiguration.FirstChild().ToElement();
  for(; child; child=child->NextSiblingElement()) {
    // If "TorsoHeight" element is found, try to read "torsoHeight" value
    if (child->ValueStr().compare("TorsoHeight") == 0) {
      bool isFore = false;
      bool isHind = false;
      double defaultTorsoHeight = 0.0;
      if (child->QueryDoubleAttribute("torsoHeight", &defaultTorsoHeight) != TIXML_SUCCESS) {
        /*
         * Note: desiredTorsoCoMHeightAboveGroundInControlFrameOffset_ is set in the constructor
         */
        std::cout << magenta << "[TorsoControlBase/loadParametersTorsoConfiguration] "
                  << red << "Warning: "
                  << blue << "could not find parameter 'torsoHeight' in section 'TorsoHeight'. Setting height to default value: "
                  << red << desiredTorsoCoMHeightAboveGroundInControlFrameOffset_
                  << def << std::endl;
      }
      else {
        desiredTorsoCoMHeightAboveGroundInControlFrameOffset_ = defaultTorsoHeight;
        std::cout << magenta << "[TorsoControlBase/loadParametersTorsoConfiguration] "
                  << blue << "Torso height is set to: "
                  << red << desiredTorsoCoMHeightAboveGroundInControlFrameOffset_
                  << def << std::endl;
      }
    }
    else {
      std::cout << magenta << "[TorsoControlBase/loadParametersTorsoConfiguration] "
                << red << "Warning: "
                << blue << "could not find section 'TorsoHeight'. Setting height to default value: "
                << red << desiredTorsoCoMHeightAboveGroundInControlFrameOffset_
                << def << std::endl;
    }

  }

  return true;
}


bool TorsoControlBase::loadParametersHipConfiguration(const TiXmlHandle &hParameterSet) {

  int iKnot;
  double t, value;
  TiXmlElement* pElem;
  std::string legFrame;

  /* Swing foot configuration*/
  pElem = hParameterSet.FirstChild("HipConfiguration").Element();
  if (!pElem) {
    printf("Could not find HipConfiguration\n");
    return false;
  }

  /**************************************************************************
   * HEIGHT
   ***************************************************************************/

  /* offset */
  pElem = hParameterSet.FirstChild("HipConfiguration").Element();
  if (!pElem) {
    printf("Could not find HipConfiguration!\n");
    return false;
  }
  TiXmlElement* child = hParameterSet.FirstChild("HipConfiguration").FirstChild().ToElement();
       for( child; child; child=child->NextSiblingElement() ){
          if (child->ValueStr().compare("HeightTrajectory") == 0) {
            bool isFore = false;
            bool isHind = false;
            double offset = 0.0;
            if (child->QueryDoubleAttribute("offset", &offset)!=TIXML_SUCCESS) {
              printf("Could not find offset!\n");
            }
            if (child->QueryBoolAttribute("fore", &isFore)==TIXML_SUCCESS) {
              if (isFore) {
                desiredTorsoForeHeightAboveGroundInWorldFrameOffset_ = offset;
                TiXmlHandle hTrajectory(child);
                if(!loadHeightTrajectory(hTrajectory,  desiredTorsoForeHeightAboveGroundInWorldFrame_)) {
                  return false;
                }
              }
            }
            if (child->QueryBoolAttribute("hind", &isHind)==TIXML_SUCCESS) {
              if (isHind) {
                desiredTorsoHindHeightAboveGroundInWorldFrameOffset_ = offset;
                TiXmlHandle hTrajectory(child);
                if(!loadHeightTrajectory(hTrajectory,  desiredTorsoHindHeightAboveGroundInWorldFrame_)) {
                  return false;
                }
              }
            }
          }

       }

       //desiredTorsoCoMHeightAboveGroundInControlFrameOffset_ =




//
//      /* front leg frame */
//      pElem = hParameterSet.FirstChild("HipConfiguration").FirstChild("Fore").Element();
//      if (!pElem) {
//        printf("Could not find HipConfiguration:Fore!\n");
//        return false;
//      }
//      TiXmlHandle hTrajectory (hParameterSet.FirstChild("HipConfiguration").FirstChild("Fore").FirstChild("HeightTrajectory"));
//      pElem = hTrajectory.Element();
//      if (!pElem) {
//        printf("Could not find HeightTrajectory!\n");
//        return false;
//      }
//      if (pElem->QueryDoubleAttribute("offset", &desiredFrameHeightOffset)!=TIXML_SUCCESS) {
//        printf("Could not find HeightTrajectory:offset!\n");
//        return false;
//      }
//      if (!loadHeightTrajectory(hTrajectory)) {
//        printf("problem\n");
//        return false;
//      }
//
//
//      /* hind leg frame */
//      pElem = hParameterSet.FirstChild("HipConfiguration").FirstChild("Hind").Element();
//      if (!pElem) {
//        printf("Could not find HipConfiguration:Hind!\n");
//        return false;
//      }
//      TiXmlHandle hTrajectory (hParameterSet.FirstChild("HipConfiguration").FirstChild("Hind").FirstChild("HeightTrajectory"));
//      pElem = hTrajectory.Element();
//      if (!pElem) {
//        printf("Could not find HeightTrajectory!\n");
//        return false;
//      }
//      if (pElem->QueryDoubleAttribute("offset", &desiredFrameHeightOffset)!=TIXML_SUCCESS) {
//        printf("Could not find HeightTrajectory:offset!\n");
//        return false;
//      }
//      if (!loadHeightTrajectory(hTrajectory)) {
//        printf("problem\n");
//        return false;
//      }


//  else {
//
//    // both leg frames
//    pElem = hParameterSet.FirstChild("HipConfiguration").FirstChild("ForeAndHind").FirstChild("HeightTrajectory").Element();
//    if (!pElem) {
//      pElem = hParameterSet.FirstChild("HipConfiguration").FirstChild("FrontAndHind").FirstChild("PitchTrajectory").Element();
//      if (!pElem) {
//        printf("Could not find HeightTrajectory or PitchTrajectory!\n");
//        return false;
//      }
//      TiXmlHandle hTrajectory (hParameterSet.FirstChild("HipConfiguration").FirstChild("FrontAndHind").FirstChild("PitchTrajectory"));
//      if (pElem->QueryDoubleAttribute("offset", &desiredFrameHeightOffset)!=TIXML_SUCCESS) {
//        printf("Could not find PitchTrajectory:offset!\n");
//        return false;
//      }
//      printf("ERRLOR: pitch trajectory not implemented!\n");
//      return false;
//
//    }
//    else {
//
//      TiXmlHandle hTrajectory (hParameterSet.FirstChild("HipConfiguration").FirstChild("FrontAndHind").FirstChild("HeightTrajectory"));
//      if (pElem->QueryDoubleAttribute("offset", &desiredFrameHeightOffset)!=TIXML_SUCCESS) {
//        printf("Could not find HeightTrajectory:offset!\n");
//        return false;
//      }
//      if (!loadHeightTrajectory(hTrajectory)) {
//        printf("problem\n");
//        return false;
//      }
//    }
//
//  }

  return true;
}

bool TorsoControlBase::loadHeightTrajectory(const TiXmlHandle &hTrajectory,  rbf::PeriodicRBF1DC1& trajectory)
{
  TiXmlElement* pElem;
  int iKnot;
  double t, value;
  std::vector<double> tValues, xValues;


  TiXmlElement* child = hTrajectory.FirstChild().ToElement();
   for( child; child; child=child->NextSiblingElement() ){
      if (child->QueryDoubleAttribute("t", &t)!=TIXML_SUCCESS) {
        printf("Could not find t of knot!\n");
        return false;
      }
      if (child->QueryDoubleAttribute("v", &value)!=TIXML_SUCCESS) {
        printf("Could not find v of knot!\n");
        return false;
      }
      tValues.push_back(t);
      xValues.push_back(value);
//      printf("t=%f, v=%f\n", t, value);
   }
   trajectory.setRBFData(tValues, xValues);


  return true;
}


} /* namespace loco */
