
#include "dogbot/LegKinematics.hh"
#include <math.h>
#include <assert.h>
#include <iostream>

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

    m_footDrop = value.get("FootDrop",m_footDrop).asFloat();
    m_footSphereRadius = value.get("FootSphereRadius",m_footSphereRadius).asFloat();

    m_jointDirections[0] = value.get("dirRoll",m_jointDirections[0]).asFloat();
    m_jointDirections[1] = value.get("dirPitch",m_jointDirections[1]).asFloat();
    m_jointDirections[2] = value.get("dirKnee",m_jointDirections[2]).asFloat();

    m_alternateSolution = value.get("alternateSolution",false).asBool();

    // Generate some default leg positions.

    if(m_name == "front_left") {
      m_legOrigin[0] = -m_bodyWidth/2.0;
      m_legOrigin[1] = m_bodyLength/2.0;
      m_legOrigin[2] = 0;
    } else if(m_name == "front_right") {
      m_legOrigin[0] = m_bodyWidth/2.0;
      m_legOrigin[1] = m_bodyLength/2.0;
      m_legOrigin[2] = 0;
    } else if(m_name == "back_left") {
      m_legOrigin[0] = -m_bodyWidth/2.0;
      m_legOrigin[1] = -m_bodyLength/2.0;
      m_legOrigin[2] = 0;
    } else if(m_name == "back_right") {
      m_legOrigin[0] = m_bodyWidth/2.0;
      m_legOrigin[1] = -m_bodyLength/2.0;
      m_legOrigin[2] = 0;
    }
    std::cout << " Name:" << m_name << "  Origin:" << m_legOrigin[0] << " " << m_legOrigin[1] << " " << m_legOrigin[2] << " " << std::endl;
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

    ret["dirRoll"] = m_jointDirections[0];
    ret["dirPitch"] = m_jointDirections[1];
    ret["dirKnee"] = m_jointDirections[2];

    ret["FootDrop"] = m_footDrop;
    ret["FootSphereRadius"] = m_footSphereRadius;

    ret["alternateSolution"] = m_alternateSolution;

    return ret;
  }


  //! Compute the joint angles given a location.
  bool LegKinematicsC::InverseVirtual(float at[3],float (&angles)[3]) const
  {
#if 1
    float x = -at[0];
    float y = -at[1];
    float z = -at[2];
#else
    float x = -at[0] - m_legOrigin[0];
    float y = -at[1] - m_legOrigin[1];
    float z = -at[2] - m_legOrigin[2];
#endif

    {
      float ta = atan2(x,z);

      angles[0] = ta;

      float xr = x;
      float zr = z;

      z = sin(ta) * xr + cos(ta) * zr - m_zoff;
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

    angles[2] = kneeTarget;

    for(int i = 0;i < 3;i++)
      angles[i] *= m_jointDirections[i];

    return true;
  }

  //! Forward kinematics for the leg.
  bool LegKinematicsC::ForwardVirtual(float angles[3],float (&at)[3]) const
  {
    for(int i = 0;i < 3;i++)
      angles[i] *= m_jointDirections[i];

    float kneeAngle = angles[2];

    float y = m_l1 * sin(angles[1]) + m_l2 * sin(angles[1] + kneeAngle);
    float z = m_l1 * cos(angles[1]) + m_l2 * cos(angles[1] + kneeAngle);

    //float xr = 0;
    at[0] =  -sin(angles[0]) * (m_zoff + z); //  + cos(angles[0]) * xr
    at[1] =  + y;
    at[2] =  -cos(angles[0]) * (m_zoff + z); // + sin(angles[0]) * xr

    //for(int i = 0;i < 3;i++) at[i] += m_legOrigin[i];

    return true;
  }

  //! Inverse kinematics for the leg
  //! Compute joint angles needed to get to a 3d position in a leg coordinate system
  //! Return true if position is reachable
  bool LegKinematicsC::InverseDirect(float at[3],float (&angles)[3]) const
  {
    if(!InverseVirtual(at,angles))
      return false;

    // Allow for 4 bar linkage
    float theta = 0;
    if(!Linkage4BarBack(angles[2] ,theta,UseAlternateSolution())) {
      return false;
    }
    angles[2] = theta - angles[1];

    return true;
  }

  //! Forward kinematics for the leg
  //! Compute the position of the foot relative to the top of the leg from the joint angles.
  bool LegKinematicsC::ForwardDirect(float angles[3],float (&at)[3]) const
  {
    float theta = angles[2] + angles[1];

    // Theta = Knee Servo
    // Psi = Knee joint
    float psi = Linkage4BarForward(theta,UseAlternateSolution());
    float anglesVirtual[3] = {angles[0],angles[1],psi};

    if(!ForwardVirtual(anglesVirtual,at))
      return false;
    return false;
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

