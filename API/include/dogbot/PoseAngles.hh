#ifndef DOGBOT_POSEANGLES_HEADER
#define DOGBOT_POSEANGLES_HEADER 1

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
  //! This is hard wired to consist of 12 joints, numbered 0 to 11

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

    //! Set leg joint angles
    void SetLegJointAngles(int leg,float pitch,float roll,float knee);

    //! Set angle for a particular joint
    void SetJoint(int jnt,float pos,float torque = 0)
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

    //! Access joint information
    float JointPosition(int jnt) const
    {
      assert(jnt >= 0 && jnt < 12);
      return m_joints[jnt].Position();
    }

    //! Access joint information
    float JointTorque(int jnt) const
    {
      assert(jnt >= 0 && jnt < 12);
      return m_joints[jnt].Torque();
    }

  protected:
    JointAngleC m_joints[12];
  };

}
#endif
