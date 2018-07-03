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
}
