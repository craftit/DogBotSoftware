
#include <cassert>
#include "dogbot/SimpleQuadrupedController.hh"

namespace DogBotN {

  // Set the leg goal position
  void SimpleQuadrupedPoseC::SetLegPosition(int legId,float x,float y,float z)
  {
    int off = legId * 3;
    assert(off >= 0);
    assert(off < 12);

    m_position[off+0] = x;
    m_position[off+1] = y;
    m_position[off+2] = z;
  }

  // Set the leg goal position
  void SimpleQuadrupedPoseC::GetLegPosition(int legId,float &x,float &y,float &z)
  {
    int off = legId * 3;
    assert(off >= 0);
    assert(off < 12);

    x = m_position[off+0];
    y = m_position[off+1];
    z = m_position[off+2];
  }

  //! Dump pose.

  void SimpleQuadrupedPoseC::Dump(std::ostream &out)
  {
    out << m_position[0];
    for(int i = 1;i < 12;i++)
      out << " " << m_position[i];
  }


  // Set the leg goal position
  void SimpleQuadrupedPoseC::SetLegPosition(int legId,const Eigen::Vector3f &at)
  {
    int off = legId * 3;
    assert(off >= 0);
    assert(off < 12);

    for(int i = 0;i < 3;i++)
      m_position[off+i] = at[i];
  }

  // Set the leg goal position
  void SimpleQuadrupedPoseC::GetLegPosition(int legId,Eigen::Vector3f &at)
  {
    int off = legId * 3;
    assert(off >= 0);
    assert(off < 12);

    for(int i = 0;i < 3;i++)
      at[i] = m_position[off+i];
  }

  // Get the leg goal position
  Eigen::Vector3f SimpleQuadrupedPoseC::LegPosition(int legId)
  {
    Eigen::Vector3f ret;
    int off = legId * 3;
    assert(off >= 0);
    assert(off < 12);

    for(int i = 0;i < 3;i++)
      ret[i] = m_position[off+i];

    return ret;
  }


  // ------------------------------------------------------

  SimpleQuadrupedControllerC::SimpleQuadrupedControllerC()
  {}

  //! Virtual destructor
  SimpleQuadrupedControllerC::~SimpleQuadrupedControllerC()
  {}

  //! Set omega
  void SimpleQuadrupedControllerC::SetOmega(float omega)
  {
    m_omega = omega;
  }

  //! Set the gait style
  bool SimpleQuadrupedControllerC::SetStyle(const std::string &styleName)
  {
    return false;
  }

  //! Do a single timestep
  bool SimpleQuadrupedControllerC::Step(float timeStep,SimpleQuadrupedPoseC &positions)
  {
    return true;
  }

}
