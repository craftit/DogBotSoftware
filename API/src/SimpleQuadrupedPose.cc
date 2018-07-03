#include "dogbot/SimpleQuadrupedController.hh"

namespace DogBotN {

  // Set the leg goal position
  void SimpleQuadrupedPoseC::SetLegPosition(int legId,float x,float y,float z)
  {
    m_feet[legId] =  Eigen::Vector3f(x,y,z);
  }

  // Set the leg goal position
  void SimpleQuadrupedPoseC::GetLegPosition(int legId,float &x,float &y,float &z) const
  {
    const Eigen::Vector3f &at = m_feet[legId];
    x = at[0];
    y = at[1];
    z = at[2];
  }

  //! Dump pose.

  void SimpleQuadrupedPoseC::Dump(std::ostream &out)
  {
    for(int i = 0;i < 4;i++) {
      out << " (" << m_feet[i] << ")";
    }
  }


  // Set the leg goal position
  void SimpleQuadrupedPoseC::SetLegPosition(int legId,const Eigen::Vector3f &at)
  {
    assert(legId >= 0 && legId < 4);
    m_feet[legId] = at;
  }

  // Set the leg goal position
  void SimpleQuadrupedPoseC::GetLegPosition(int legId,Eigen::Vector3f &at) const
  {
    assert(legId >= 0 && legId < 4);
    at = m_feet[legId];
  }


}
