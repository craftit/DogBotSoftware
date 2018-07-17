#ifndef DOGBOT_QUADRUPEDCONTROLLER_HEADER
#define DOGBOT_QUADRUPEDCONTROLLER_HEADER 1

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Geometry>
#include "dogbot/SimpleQuadrupedPose.hh"

namespace DogBotN {

  //! Simple Gait generator base class.

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
