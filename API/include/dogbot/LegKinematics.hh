#ifndef DOGBOT_LEGKINEMATICS_HEADER
#define DOGBOT_LEGKINEMATICS_HEADER 1

namespace DogBotN {


  //! This computes the position of the end effector given the end effector

  // l1 = Upper leg length.
  // l2 = Lower leg length

  // Coordinates:
  //  x = forward/back
  //  y = sideways
  //  z = Height above ground
  //
  // Angles:
  //  0 - Pelvis
  //  1 - Hip
  //  2 - Knee

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

    //! 4 bar linkage angle forward.
    float Linkage4BarForward(float theta,bool solution2 = false) const;

    //! 4 bar linkage angle backward,
    // Returns true if angle exists.
    bool Linkage4BarBack(float psi,float &ret,bool solution2 = false) const;

    // See: https://synthetica.eng.uci.edu/mechanicaldesign101/McCarthyNotes-2.pdf
    bool Linkage4Bar(float theta,float a,float b,float g,float h,float &psi,bool solution2 = false) const;

    // Compute the speed ratio at the given input angle.
    float LinkageSpeedRatio(float theta,float psi);

  protected:

    float m_l1 = 0.361; // Upper leg length
    float m_l2 = 0.31;
    float m_zoff = 0.08;

    float m_linkA = 0.032; // Hip CAD: 0.032
    float m_linkB = 0.04;  // Knee CAD: 0.04
    float m_linkH = 0.363; // Push rod 0.363
  };

}

#endif
