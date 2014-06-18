/*
 * TerrainModelPerceptedPlane.hpp
 *
 *  Created on: Apr 4, 2014
 *      Author: gech
 */

#ifndef LOCO_TERRAINMODELPerceptedPLANE_HPP_
#define LOCO_TERRAINMODELPerceptedPLANE_HPP_

#include "loco/common/TerrainModelBase.hpp"
#include "robotUtils/loggers/Logger.hpp"


namespace loco {

class TerrainModelPerceptedPlane: public TerrainModelBase {
 private:
   Position pointOnPlane;
   Vector normalPlane;

   /*!
    * A 3x3 float Matrix, which is used for the exponential forgetting problem.
    */
   Eigen::Matrix3d omega;

   //exponential forgetting coefficient
   const double lambda=0.7;

   double slopeAngle;
   double rollAngle;

 public:
  TerrainModelPerceptedPlane();
  virtual ~TerrainModelPerceptedPlane();

  /*!
   * Gets the surface normal of the terrain at a certain position.
   * @param[in] position the place to get the surface normal from (in the world frame)
   * @param[out] normal the surface normal (in the world frame)
   * @return true if successful, false otherwise
   */
  virtual bool getNormal(const loco::Position& position, loco::Vector& normal) const;
  virtual const loco::Vector& getNormal()const;

  /*
   * returns a point on the predictedPlane
   */
  virtual const Position& getPointOnPlane() const;

  /*!
   * Gets the height of the terrain at the coordinate (position.x(), position.y())
   * (in world frame) and sets the position.z() as the height (in world frame).
   * @param[in/out] position.z() is set as the height of the terrain at
   *                (position.x(), position.y()) (in world frame)
   * @return true if successful, false otherwise
   */
  virtual bool getHeight(loco::Position& position) const;

  virtual void setHeight(double height);

  /*!
     * Updates the plane, which percepts the ground through given points
     * It is an exponential forgetting least square algorithm
     * The plane is presented by the formula: ax+by+z-d=0
     *@returns true if successful, false otherwize
     */
    virtual bool updatePlane(const Position& position);

    /*
     * Returns a reference to the slope angle
     */
    virtual const double& getSlopeAngle() const;

    /*
     * Returns a reference to the roll angle
     */
    virtual const double& getRollAngle() const;

    /*!
     * Updates the slope angle
     */
    virtual void updateAngles(const RotationQuaternion& bodyHeadingInWorldFrame);

    /*
     * Logger for the angles
     */
    virtual bool addSlopeAngleLogger(double * var);
    virtual bool addRollAngleLogger(double * var);

    virtual bool initialize(double dt);

 protected:
  double height_;

};

} /* namespace loco */

#endif /* LOCO_TERRAINMODELPerceptedPLANE_HPP_ */
