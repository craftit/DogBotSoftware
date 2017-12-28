
#include "dogbot/LegKinematics.hh"
#include <math.h>
#include <assert.h>

namespace DogBotN {

  static float Sqr(float val)
  { return val * val; }

  LegKinematicsC::LegKinematicsC()
  {}

  //! Construct from a json object
  LegKinematicsC::LegKinematicsC(const Json::Value &value)
  {
    ConfigureFromJSON(value);
  }

  //! Create
  LegKinematicsC::LegKinematicsC(float l1,float l2)
   : m_l1(l1),
     m_l2(l2)
  {}

  //! Configure from JSON
  bool LegKinematicsC::ConfigureFromJSON(const Json::Value &value)
  {
    m_name = value.get("name","").asString();
    m_l1 = value.get("l1",m_l1).asFloat();
    m_l2 = value.get("l2",m_l2).asFloat();
    m_zoff = value.get("ZOffset",m_zoff).asFloat();
    m_linkA = value.get("LinkA",m_linkA).asFloat();
    m_linkB = value.get("LinkB",m_linkB).asFloat();
    m_linkH = value.get("LinkH",m_linkH).asFloat();

    return true;
  }

  //! Get the servo configuration as JSON
  Json::Value LegKinematicsC::ConfigAsJSON() const
  {
    Json::Value ret;

    ret["name"] = m_name;
    ret["l1"] = m_l1;
    ret["l2"] = m_l2;
    ret["ZOffset"] = m_zoff;
    ret["LinkA"] = m_linkA;
    ret["LinkB"] = m_linkB;
    ret["LinkH"] = m_linkH;

    return ret;
  }


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

    float kneeTarget = M_PI - acos(ac2);

    float servoAngle = 0;
    if(!Linkage4BarBack(kneeTarget,servoAngle))
      return false;
    angles[2] = servoAngle;

    return true;
  }

  //! Forward kinematics for the leg.
  bool LegKinematicsC::Forward(const float (&angles)[3],float (&at)[3]) const
  {
    float kneeAngle = Linkage4BarForward(angles[2]);

    float y = m_l1 * sin(angles[1]) + m_l2 * sin(angles[1] + kneeAngle);
    float z = m_l1 * cos(angles[1]) + m_l2 * cos(angles[1] + kneeAngle);

    float xr = 0;
    at[0] = cos(angles[0]) * xr + sin(angles[0]) * (m_zoff + z);
    at[1] = y;
    at[2] = sin(angles[0]) * xr + cos(angles[0]) * (m_zoff + z);

    return true;
  }

  //! 4 bar linkage angle backward,
  // Returns true if angle exists.
  bool LegKinematicsC::Linkage4BarBack(float psi,float &theta,bool solution2) const
  {
    float val = 0;
    if(!Linkage4Bar(M_PI - psi,m_linkB,m_linkA,m_l1,m_linkH,val,solution2))
      return false;
    theta = M_PI - val;
    return true;
  }

  float LegKinematicsC::Linkage4BarForward(float theta,bool solution2) const
  {
    float ret = 0;
    bool ok = Linkage4Bar(theta,m_linkA,m_linkB,m_l1,m_linkH,ret,solution2);
    //assert(ok);
    return ret; // Psi
  }

  bool LegKinematicsC::Linkage4Bar(
      float theta,
      float a,float b,float g,float h,
      float &psi,
      bool solution2
      ) const
  {
    float At = 2 * b * g - 2 * a * b * cos(theta);
    float Bt = -2 * a * b * sin(theta);
    float Ct = a*a + b * b + g * g -h * h - 2 * a * g * cos(theta);

    float delta = atan(Bt/At);
    float cosAngle = -Ct / sqrt(At * At + Bt * Bt);
    if(cosAngle > 1.0 || cosAngle < -1.0)
      return false;
    psi = delta + (solution2 ? -1 : 1) * acos(cosAngle);
    return true;
  }

  // Compute the speed ratio at the given input angle.

  float LegKinematicsC::LinkageSpeedRatio(float theta,float psi) const
  {
    float a = m_linkA;
    float b = m_linkB;
    float g = m_l1;
    float h = m_linkH;

    float result =
        -(- a * b * cos(psi) * sin(theta) - b * g * sin(psi) + a * b * cos(theta) * sin(psi)) /
        (a * g * sin(theta) + a * b * cos(psi) * sin(theta) - a * b * cos(theta) * sin(psi) );

    return result;
  }


}

