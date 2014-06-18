/*
 * GeneralizedStateStarlETH.cpp
 *
 *  Created on: Apr 4, 2014
 *      Author: gech
 */

#include "loco/common/GeneralizedStateStarlETH.hpp"

namespace loco {

GeneralizedStateStarlETH::GeneralizedStateStarlETH() :
    generalizedCoordinates_(GeneralizedCoordinates::Zero()),
    generalizedVelocities_(GeneralizedVelocities::Zero()),
    generalizedAccelerations_(GeneralizedAccelerations::Zero())
{


}

GeneralizedStateStarlETH::~GeneralizedStateStarlETH() {

}

void GeneralizedStateStarlETH::setPositionWorldToBaseInWorldFrame(const Position& worldToBaseInWorldFrame) {
  generalizedCoordinates_.block<3,1>(0,0) = worldToBaseInWorldFrame.toImplementation();
}

const GeneralizedCoordinates& GeneralizedStateStarlETH::getGeneralizedCoordinates() const {
  return generalizedCoordinates_;
}

const GeneralizedVelocities& GeneralizedStateStarlETH::getGeneralizedVelocities() const
{
  return generalizedVelocities_;
}

const GeneralizedAccelerations& GeneralizedStateStarlETH::getGeneralizedAccelerations() const
{
  return generalizedAccelerations_;
}

bool GeneralizedStateStarlETH::loadParameters(const TiXmlHandle& handle) {
  TiXmlElement* pElem;
  TiXmlHandle hStarlETH(handle.FirstChild("GeneralizedState").FirstChild("StarlETH"));
  pElem = hStarlETH.Element();
  if (!pElem) {
    printf("Could not find GeneralizedState:StarlETH\n");
    return false;
  }


  /* generalized coordinates */

  TiXmlHandle hGeneralizedCoordinates(hStarlETH.FirstChild("GeneralizedCoordinates"));
  pElem = hGeneralizedCoordinates.Element();
  if (!pElem) {
    printf("Could not find GeneralizedCoordinates\n");
    return false;
  }

  pElem = hGeneralizedCoordinates.FirstChild("Torso").FirstChild("Position").ToElement();
  if (!pElem) {
    printf("Could not find GeneralizedCoordinates:Torso:Position\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("x", &generalizedCoordinates_(0))!=TIXML_SUCCESS) {
    printf("Could not find Position:x\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("y", &generalizedCoordinates_(1))!=TIXML_SUCCESS) {
    printf("Could not find Position:y\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("z", &generalizedCoordinates_(2))!=TIXML_SUCCESS) {
    printf("Could not find Position:z\n");
    return false;
  }


  pElem = hGeneralizedCoordinates.FirstChild("Torso").FirstChild("Orientation").ToElement();
  if (!pElem) {
    printf("Could not find GeneralizedCoordinates:Torso:Orientation\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("w", &generalizedCoordinates_(3))!=TIXML_SUCCESS) {
    printf("Could not find Orientation:w\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("x", &generalizedCoordinates_(4))!=TIXML_SUCCESS) {
    printf("Could not find Orientation:x\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("y", &generalizedCoordinates_(5))!=TIXML_SUCCESS) {
    printf("Could not find Orientation:y\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("z", &generalizedCoordinates_(6))!=TIXML_SUCCESS) {
    printf("Could not find Orientation:z\n");
    return false;
  }

  double hAA, hFE, kFE;
  pElem = hGeneralizedCoordinates.FirstChild("Leg").FirstChild("HAA").ToElement();
  if (!pElem) {
    printf("Could not find GeneralizedCoordinates:Leg:HAA\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("theta", &hAA)!=TIXML_SUCCESS) {
    printf("Could not find theta!\n");
    return false;
  }
  pElem = hGeneralizedCoordinates.FirstChild("Leg").FirstChild("HFE").ToElement();
  if (!pElem) {
    printf("Could not find GeneralizedCoordinates:Leg:HFE\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("theta", &hFE)!=TIXML_SUCCESS) {
    printf("Could not find theta!\n");
    return false;
  }
  pElem = hGeneralizedCoordinates.FirstChild("Leg").FirstChild("KFE").ToElement();
  if (!pElem) {
    printf("Could not find GeneralizedCoordinates:Leg:KFE\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("theta", &kFE)!=TIXML_SUCCESS) {
    printf("Could not find theta!\n");
    return false;
  }
  setJointPositionsSymmetricToLeftForeLeg(hAA, hFE, kFE);




  /* generalized velocities */

  TiXmlHandle hGeneralizedVelocities(hStarlETH.FirstChild("GeneralizedVelocities"));
    pElem = hGeneralizedVelocities.Element();
    if (!pElem) {
      printf("Could not find GeneralizedVelocities\n");
      return false;
    }

    pElem = hGeneralizedVelocities.FirstChild("Torso").FirstChild("LinearVelocity").ToElement();
    if (!pElem) {
      printf("Could not find GeneralizedVelocities:Torso:LinearVelocity\n");
      return false;
    }

    if (pElem->QueryDoubleAttribute("x", &generalizedVelocities_(0))!=TIXML_SUCCESS) {
      printf("Could not find LinearVelocity:x\n");
      return false;
    }

    if (pElem->QueryDoubleAttribute("y", &generalizedVelocities_(1))!=TIXML_SUCCESS) {
      printf("Could not find LinearVelocity:y\n");
      return false;
    }

    if (pElem->QueryDoubleAttribute("z", &generalizedVelocities_(2))!=TIXML_SUCCESS) {
      printf("Could not find LinearVelocity:z\n");
      return false;
    }


    pElem = hGeneralizedVelocities.FirstChild("Torso").FirstChild("LocalAngularVelocity").ToElement();
    if (!pElem) {
      printf("Could not find GeneralizedCoordinates:Torso:LocalAngularVelocity\n");
      return false;
    }


    if (pElem->QueryDoubleAttribute("x", &generalizedVelocities_(3))!=TIXML_SUCCESS) {
      printf("Could not find Orientation:x\n");
      return false;
    }

    if (pElem->QueryDoubleAttribute("y", &generalizedVelocities_(4))!=TIXML_SUCCESS) {
      printf("Could not find Orientation:y\n");
      return false;
    }

    if (pElem->QueryDoubleAttribute("z", &generalizedVelocities_(5))!=TIXML_SUCCESS) {
      printf("Could not find Orientation:z\n");
      return false;
    }


    pElem = hGeneralizedVelocities.FirstChild("Leg").FirstChild("HAA").ToElement();
    if (!pElem) {
      printf("Could not find GeneralizedVelocities:Leg:HAA\n");
      return false;
    }
    if (pElem->QueryDoubleAttribute("theta", &hAA)!=TIXML_SUCCESS) {
      printf("Could not find theta!\n");
      return false;
    }
    pElem = hGeneralizedVelocities.FirstChild("Leg").FirstChild("HFE").ToElement();
    if (!pElem) {
      printf("Could not find GeneralizedVelocities:Leg:HFE\n");
      return false;
    }
    if (pElem->QueryDoubleAttribute("theta", &hFE)!=TIXML_SUCCESS) {
      printf("Could not find theta!\n");
      return false;
    }
    pElem = hGeneralizedVelocities.FirstChild("Leg").FirstChild("KFE").ToElement();
    if (!pElem) {
      printf("Could not find GeneralizedVelocities:Leg:KFE\n");
      return false;
    }
    if (pElem->QueryDoubleAttribute("theta", &kFE)!=TIXML_SUCCESS) {
      printf("Could not find theta!\n");
      return false;
    }
    setJointVelocitiesSymmetricToLeftForeLeg(hAA, hFE, kFE);

  return true;
}

void GeneralizedStateStarlETH::setJointPositionsSymmetricToLeftForeLeg(double angleHAA, double angleHFE, double angleKFE) {
  generalizedCoordinates_(7) = angleHAA;
  generalizedCoordinates_(8) = angleHFE;
  generalizedCoordinates_(9) = angleKFE;

  generalizedCoordinates_(10) = -angleHAA;
  generalizedCoordinates_(11) = angleHFE;
  generalizedCoordinates_(12) = angleKFE;

  generalizedCoordinates_(13) = angleHAA;
  generalizedCoordinates_(14) = -angleHFE;
  generalizedCoordinates_(15) = -angleKFE;


  generalizedCoordinates_(16) = -angleHAA;
  generalizedCoordinates_(17) = -angleHFE;
  generalizedCoordinates_(18) = -angleKFE;
}

void GeneralizedStateStarlETH::setJointVelocitiesSymmetricToLeftForeLeg(double velocityHAA, double velocityHFE, double velocityKFE) {
  generalizedVelocities_(6) = velocityHAA;
  generalizedVelocities_(7) = velocityHFE;
  generalizedVelocities_(8) = velocityKFE;

  generalizedVelocities_(9) = -velocityHAA;
  generalizedVelocities_(10) = velocityHFE;
  generalizedVelocities_(11) = velocityKFE;

  generalizedVelocities_(12) = velocityHAA;
  generalizedVelocities_(13) = -velocityHFE;
  generalizedVelocities_(14) = -velocityKFE;

  generalizedVelocities_(15) = -velocityHAA;
  generalizedVelocities_(16) = -velocityHFE;
  generalizedVelocities_(17) = -velocityKFE;
}

void GeneralizedStateStarlETH::setJointPositionsForLeg(int iLeg, const Eigen::Vector3d& angles) {
  generalizedCoordinates_.block<3,1>(7+iLeg*3,0) = angles;

}

} /* namespace loco */

