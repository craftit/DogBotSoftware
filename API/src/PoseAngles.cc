#include "dogbot/PoseAngles.hh"
#include "dogbot/DogBotKinematics.hh"

namespace DogBotN {

  std::string PoseAnglesC::JointName(int leg,int legJnt)
  {
    std::string ret = DogBotKinematicsC::LegNames()[leg];
    switch(legJnt) {
    case 0: ret += "_roll"; break;
    case 1: ret += "_pitch"; break;
    case 2: ret += "_knee"; break;
    default:
      assert(0 && "Joint number out of range 0 to 2");
      return "*internal error*";
    }
    return ret;
  }

  //! Set leg joint angles
  void PoseAnglesC::SetLegJointAngles(
      int leg,
      const Eigen::Vector3f &vec
      )
  {
    for(int i = 0;i < 3;i++) {
      m_joints[JointId(leg,i)] = JointAngleC(vec[i],0);
    }
  }

  //! Set leg joint angles
  void PoseAnglesC::SetLegJointAngles(int leg,float pitch,float roll,float knee)
  {
    SetLegJointAngles(leg,Eigen::Vector3f(pitch,roll,knee));
  }


}
