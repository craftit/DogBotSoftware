#ifndef DOGBOT_JOINTPOSE_HEADER
#define DOGBOT_JOINTPOSE_HEADER 1

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Geometry>

namespace DogBotN {

  //! Joint position information

  class JointAngleC
  {
  public:
    //! Default constructor
    // It will have a position 0 and torque 0
    JointAngleC()
    {}

    //! Joint position angle in radians, and torque in Newton meters.
    // If the torque is unknown set to 0.
    JointAngleC(float position,float torque)
     : m_position(position),
       m_torque(torque)
    {}

    //! Joint angle in radians
    float Position() const
    { return m_position; }

    //! Torque estimate in Newton meters.
    float Torque() const
    { return m_torque; }

  protected:
    float m_position = 0;
    float m_torque = 0;
  };

  //! Robot pose in terms of joint angles and expected torques.

  class PoseAnglesC
  {
  public:
    PoseAnglesC()
    {}

    // Identify joint in array.
    // Leg: as DogBotAPIC::LegNames();
    // legJnt: 0=Roll, 1=Pitch, 2=Knee
    static int JointId(int leg,int legJnt)
    {
      assert(leg >= 0 && leg < 4);
      assert(legJnt >= 0 && legJnt < 3);
      return leg * 3 + legJnt;
    }

    //! Name of joint for the given leg and joint number.
    static std::string JointName(int leg,int legJnt);

    //! Set leg joint angles
    void SetLegJointAngles(int leg,const Eigen::Vector3f &vec);

    //! Set angle for a particular joint
    void SetJointAngle(int jnt,float pos,float torque)
    {
      assert(jnt >= 0 && jnt < 12);
      m_joints[jnt] = JointAngleC(pos,torque);
    }

    //! Access joint information
    const JointAngleC &JointAngle(int jnt) const
    {
      assert(jnt >= 0 && jnt < 12);
      return m_joints[jnt];
    }

  protected:
    JointAngleC m_joints[12];
  };

}
#endif