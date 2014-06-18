/*
 * TerrainModelPerceptedPlane.cpp
 *
 *  Created on: Apr 4, 2014
 *      Author: gech
 */

#include "loco/common/TerrainModelPerceptedPlane.hpp"

namespace loco {

TerrainModelPerceptedPlane::TerrainModelPerceptedPlane() :
    TerrainModelBase(),
    height_(0.0)
{
    normalPlane.x()=0.0;
    normalPlane.y()=0.0;
    normalPlane.z()=1.0;

    pointOnPlane.setZero();
    slopeAngle=0;
    rollAngle=0;
    addRollAngleLogger(&rollAngle);
    addSlopeAngleLogger(&slopeAngle);

    //omega calculated in matlab
    //omega=inv(H*H') with H=[1 1 1 1; -1 1 0 0; 0 0 1 -1]
    omega.setZero();
    omega(0,0)=0.25;
    omega(1,1)=0.25;
    omega(2,2)=0.25;

    //std::cout <<std::endl <<omega <<std::endl;

}

bool TerrainModelPerceptedPlane::initialize(double dt) {
  normalPlane.x()=0.0;
    normalPlane.y()=0.0;
    normalPlane.z()=1.0;

    pointOnPlane.setZero();
    slopeAngle=0;
    rollAngle=0;
    addRollAngleLogger(&rollAngle);
    addSlopeAngleLogger(&slopeAngle);

    //omega calculated in matlab
    //omega=inv(H*H') with H=[1 1 1 1; -1 1 0 0; 0 0 1 -1]
    omega.setZero();
    omega(0,0)=0.25;
    omega(1,1)=0.25;
    omega(2,2)=0.25;

  return true;
}

TerrainModelPerceptedPlane::~TerrainModelPerceptedPlane() {

}

bool TerrainModelPerceptedPlane::getNormal(const loco::Position& position, loco::Vector& normal) const
{
  normal = normalPlane;
  return true;
}

const loco::Vector& TerrainModelPerceptedPlane::getNormal() const
{
  return normalPlane;
}

const Position& TerrainModelPerceptedPlane::getPointOnPlane() const
{
   return  pointOnPlane;

}

bool TerrainModelPerceptedPlane::getHeight(loco::Position& position) const
{
  /* If plane already implemented and given by a point (pointOnPlane) and the normal (normalPlane) vector */
    if (normalPlane.z() != 0) {
      position.z() = pointOnPlane.z()-((position.x()-pointOnPlane.x())*normalPlane.x()+ (position.y()-pointOnPlane.y())*normalPlane.y()) / normalPlane.z();
    }
    else {
     position.z() = 0.0;
     return false;
    }

    return true;
}

    bool TerrainModelPerceptedPlane::updatePlane(const Position& position)
    {
      //get the measured data and the previous parameter
       Eigen::Vector3d h(1.0, -position.x(), -position.y());
       double y=position.z();
       double d=pointOnPlane.dot(normalPlane);
       //std::cout <<"d "<< d << std::endl;
       Eigen::Vector3d parameter(d, normalPlane.x(), normalPlane.y());
       double c=h.transpose()*omega*h;
       //std::cout <<"c "<< c << std::endl;

       //update of the parameter
       parameter=parameter+1/(lambda+c)*omega*h*(y-h.transpose()*parameter);

       //update omega
       Eigen::Matrix3d identityMatrix=identityMatrix.setIdentity();
       //std::cout <<"identity Matrix "<< identityMatrix << std::endl;
       omega=1/lambda*omega*(identityMatrix-(1/(lambda+c)*h*h.transpose()*omega));

     //  get the normalPlane and pointOnPlane from the new parameter
     //  using the formula p=d*n and n=(a,b,c)
       normalPlane.x()=parameter.y();
       normalPlane.y()=parameter.z();


       pointOnPlane.x()=position.x();
       pointOnPlane.y()=position.y();
       pointOnPlane.z()=parameter.x()-normalPlane.x()*position.x()-normalPlane.y()*position.y();

//       std::cout <<"normalPlane and pointOnPlane  " <<std::endl;
//       std::cout <<normalPlane <<std::endl <<std::endl;
//       std::cout <<pointOnPlane<<std::endl;
//       std::cout <<"Omega"<<std::endl << omega <<std::endl;

      return true;
    }


void TerrainModelPerceptedPlane::setHeight(double height) {
  height_ = height;
}

const double& TerrainModelPerceptedPlane::getSlopeAngle() const
{
  return slopeAngle;
}

const double& TerrainModelPerceptedPlane::getRollAngle() const
{
  return rollAngle;
}

void TerrainModelPerceptedPlane::updateAngles(const RotationQuaternion& bodyHeadingInWorldFrame)
{
  EulerAnglesZyx eulerAnglesBodyHeadingInWorldFrame(bodyHeadingInWorldFrame);
  if (bodyHeadingInWorldFrame.z()<0)
  {
    eulerAnglesBodyHeadingInWorldFrame.setYaw(eulerAnglesBodyHeadingInWorldFrame.yaw()-M_PI);
  }

  //there is a bug in rotating the normal vector
//  std::cout <<"yaw angle: " <<eulerAnglesBodyHeadingInWorldFrame.yaw() << std::endl;

  Eigen::Vector3d normalPlaneInBaseDirection;
  normalPlaneInBaseDirection.z()=normalPlane.z();
  normalPlaneInBaseDirection.x()=cos(eulerAnglesBodyHeadingInWorldFrame.yaw())*normalPlane.x()-sin(eulerAnglesBodyHeadingInWorldFrame.yaw())*normalPlane.y();
  normalPlaneInBaseDirection.y()=sin(eulerAnglesBodyHeadingInWorldFrame.yaw())*normalPlane.x()+cos(eulerAnglesBodyHeadingInWorldFrame.yaw())*normalPlane.y();
  normalPlaneInBaseDirection=normalPlaneInBaseDirection/normalPlaneInBaseDirection.norm();
//  std::cout << "Normalvektor in BaseDirection: " << normalPlaneInBaseDirection;
//  std::cout << std::endl;

  slopeAngle=atan(-normalPlaneInBaseDirection.x()/normalPlaneInBaseDirection.z());
//  std::cout << "slopeAngle: " << slopeAngle*180/M_PI << std::endl;
  rollAngle=atan(normalPlaneInBaseDirection.y()/normalPlaneInBaseDirection.z());
//  std::cout << "rollAngle: " << rollAngle*180/M_PI << std::endl;

}

bool TerrainModelPerceptedPlane::addSlopeAngleLogger(double * var)
{
  robotUtils::addDoubleToLog(var, "DesiredSlopeAngle");
  robotUtils::updateLogger(true);
  return true;
}

bool TerrainModelPerceptedPlane::addRollAngleLogger(double * var)
{
  robotUtils::addDoubleToLog(var, "DesiredRollAngle");
  robotUtils::updateLogger(true);
  return true;
}

} /* namespace loco */
