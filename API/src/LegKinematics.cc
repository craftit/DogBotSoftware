
#include "dogbot/LegKinematics.hh"
#include <math.h>

namespace DogBotN {

  static float Sqr(float val)
  { return val * val; }

  LegKinematicsC::LegKinematicsC()
  {}

  //! Create
  LegKinematicsC::LegKinematicsC(float l1,float l2)
   : m_l1(l1),
     m_l2(l2)
  {}

  //! Compute the joint angles given a location.
  bool LegKinematicsC::Inverse(const float (&at)[3],float (&angles)[3]) const
  {
    float x = at[0];
    float y = -at[1];
    float z = at[2];

    {
      float ta = atan2(x,z);

      angles[0] = ta;

      float xr = x;
      float zr = z;

      z = sin(ta) * xr + cos(ta) * zr - m_zoff;
      //RavlDebug("x:%f z:%f ",x,z);
    }


    float l2 = Sqr(z) + Sqr(y);
    float l = sqrt(l2);
    float a1 = atan2(y,z);

    double ac1 = (l2 + Sqr(m_l1) - Sqr(m_l2))/(2 * l * m_l1);
    if(ac1 < -1 || ac1 > 1)
      return false;

    angles[1] = -1 *(a1 + acos(ac1));

    double ac2 = (Sqr(m_l2) + Sqr(m_l1) -l2)/(2 * m_l2 * m_l1);
    if(ac2 < -1 || ac2 > 1)
      return false;

    angles[2] = M_PI - acos(ac2);

    return true;
  }

  //! Forward kinematics for the leg.
  bool LegKinematicsC::Forward(const float (&angles)[3],float (&at)[3]) const
  {
    float y = m_l1 * sin(angles[1]) + m_l2 * sin(angles[1] + angles[2]);
    float z = m_l1 * cos(angles[1]) + m_l2 * cos(angles[1] + angles[2]);

    float xr = 0;
    at[0] = cos(angles[0]) * xr + sin(angles[0]) * (m_zoff + z);
    at[1] = y;
    at[2] = sin(angles[0]) * xr + cos(angles[0]) * (m_zoff + z);

    //RavlDebug("Len:%f ",Sqrt(Sqr(at[1]) + Sqr(at[2])));

    return true;
  }

}

