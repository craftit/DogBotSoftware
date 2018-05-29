#ifndef DOGBOT_QUADRUPEDCONTROLLER_HEADER
#define DOGBOT_QUADRUPEDCONTROLLER_HEADER 1

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Geometry>

namespace DogBotN {

  // Cartesian positions for feet.
  //
  //  0-Front left  1-Front right
  //  2-Rear left   3-Read right

  class SimpleQuadrupedPoseC
  {
  public:
    // Set the leg goal position
    void SetLegPosition(int legId,float x,float y,float z);

    // Get the leg goal position
    void GetLegPosition(int legId,float &x,float &y,float &z);

    // Set the leg goal position
    void SetLegPosition(int legId,const Eigen::Vector3f &at);

    // Get the leg goal position
    void GetLegPosition(int legId,Eigen::Vector3f &at);

    // Get the leg goal position
    Eigen::Vector3f LegPosition(int legId);

    //! Dump pose
    void Dump(std::ostream &out);

    float m_position[12];
  };

  //! Simple Gate generator base class.

  class SimpleQuadrupedControllerC
  {
  public:
    SimpleQuadrupedControllerC();

    //! Virtual destructor
    virtual ~SimpleQuadrupedControllerC();

    //! Set omega
    virtual void SetOmega(float omega);

    //! Do a single timestep
    virtual bool Step(float timeStep,SimpleQuadrupedPoseC &pose);

    //! Set the gait style
    virtual bool SetStyle(const std::string &styleName);

  protected:
    float m_omega = 2;  //!< Radians / second cycle speed
  };
}

#endif
