#ifndef DOGBOT_SIMPLEQUADRUPEDPOSE_HEADER
#define DOGBOT_SIMPLEQUADRUPEDPOSE_HEADER 1

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Geometry>

namespace DogBotN {

  // Cartesian positions for feet in body a coordinate system
  //
  //  0-Front left  1-Front right
  //  2-Rear left   3-Read right

  class SimpleQuadrupedPoseC
  {
  public:
    // Set the leg goal position
    void SetLegPosition(int legId,float x,float y,float z);

    // Get the leg goal position
    void GetLegPosition(int legId,float &x,float &y,float &z) const;

    // Set the leg goal position
    void SetLegPosition(int legId,const Eigen::Vector3f &at);

    // Get the leg goal position
    void GetLegPosition(int legId,Eigen::Vector3f &at) const;

    // Get the leg goal position
    const Eigen::Vector3f &FootPosition(int legId) const {
      assert(legId >= 0 && legId < 4);
      return m_feet[legId];
    }

    //! Dump pose
    void Dump(std::ostream &out);

    Eigen::Vector3f m_feet[4];
  };

}

#endif
