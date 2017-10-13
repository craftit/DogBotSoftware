#ifndef DOGBOT_LEGKINEMATICS_HEADER
#define DOGBOT_LEGKINEMATICS_HEADER 1

namespace ReasonN {
  namespace DogbotN {


    //! This computes the position of the end effector given the end effector

    // l1 = Upper leg length.
    // l2 = Lower leg length

    class LegKinematicsC
    {
    public:
      LegKinematicsC();

      //! Create
      LegKinematicsC(float l1,float l2);

      //! Compute the joint angles given a location.
      bool Inverse(const float (&position)[3],float (&angles)[3]) const;

      //! Forward kinematics for the leg.
      bool Forward(const float (&angles)[3],float (&position)[3]) const;

    protected:
      float m_l1 = 1.0;
      float m_l2 = 1.0;
      float m_zoff = 0.08;
    };

  }
}

#endif
