
#include "dogbot/LegKinematics.hh"
#include <math.h>
#include <assert.h>
#include <iostream>

#define DODEBUG 0
#if DODEBUG
#define ONDEBUG(x) x
#else
#define ONDEBUG(x)
#endif

namespace DogBotN {

  static float Sqr(float val)
  { return val * val; }

  LegKinematicsC::LegKinematicsC()
  {
    Init();
  }

  //! Construct from a json object
  LegKinematicsC::LegKinematicsC(const Json::Value &value)
  {
    ConfigureFromJSON(value);
  }

  //! Create
  LegKinematicsC::LegKinematicsC(float l1,float l2)
   : m_l1(l1),
     m_l2(l2)
  {
    Init();
    ONDEBUG(std::cout << " Name:" << m_name << "  Origin:" << m_legOrigin[0] << " " << m_legOrigin[1] << " " << m_legOrigin[2] << " " << std::endl);
    ONDEBUG(std::cout << "  Leg: L1=" << m_l1 << " L2=" << m_l2 << " linkA=" << m_linkA << " linkB=" << m_linkB << " LinkH=" << m_linkH << std::endl);
  }

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
    ONDEBUG(std::cout << " Name:" << m_name << "  Origin:" << m_legOrigin[0] << " " << m_legOrigin[1] << " " << m_legOrigin[2] << " " << std::endl);
    ONDEBUG(std::cout << "  Leg: L1=" << m_l1 << " L2=" << m_l2 << " linkA=" << m_linkA << " linkB=" << m_linkB << " LinkH=" << m_linkH << std::endl);
    Init();
    return true;
  }

  void LegKinematicsC::Init()
  {
    m_minExtension = MinExtension();
    m_maxExtension = MaxExtension();
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


  //! Inverse kinematics for the leg using a virtual joint for the knee
  //! Compute joint angles needed to get to a 3d position in a leg coordinate system
  //! Return true if position is reachable
  bool LegKinematicsC::InverseVirtual(const Eigen::Vector3f &at, Eigen::Vector3f &angles) const
  {
    bool ret = true;

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

      z = sin(ta) * x + cos(ta) * z - m_zoff;
    }

    float l2 = Sqr(z) + Sqr(y);
    double ac1 = (l2 + Sqr(m_l1) - Sqr(m_l2))/(2 * sqrt(l2) * m_l1);
    if(ac1 < -1 || ac1 > 1) {
      //std::cerr << "No pitch solution." << std::endl;
      return false;
    }

    float a1 = atan2(y,z);
    angles[1] = -1 *(a1 + acos(ac1));

    double ac2 = (Sqr(m_l2) + Sqr(m_l1) -l2)/(2 * m_l2 * m_l1);
    if(ac2 < -1 || ac2 > 1) {
      //std::cerr << "No knee solution." << std::endl;
      return false;
    }

    float kneeTarget = M_PI - acos(ac2);

    angles[2] = kneeTarget;

    for(int i = 0;i < 3;i++)
      angles[i] *= m_jointDirections[i];

    return ret;
  }

  //! Forward kinematics for the leg using a virtual joint for the knee
  //! Compute the position of the foot relative to the top of the leg from the joint angles.
  bool LegKinematicsC::ForwardVirtual(const Eigen::Vector3f &anglesIn,Eigen::Vector3f &at) const
  {
    Eigen::Vector3f angles;

    for(int i = 0;i < 3;i++)
      angles[i] = anglesIn[i] * m_jointDirections[i];

    float y = m_l1 * sin(angles[1]) + m_l2 * sin(angles[1] + angles[2]);
    float z = m_l1 * cos(angles[1]) + m_l2 * cos(angles[1] + angles[2]);

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
  bool LegKinematicsC::InverseDirect(const Eigen::Vector3f &at,Eigen::Vector3f &angles) const
  {
    bool ret = true;
    Eigen::Vector3f target = at;
#if 1
    // Set target to closest we can actually reach.
    float ext = at.norm();
    if(ext < m_minExtension) {
      std::cerr << "Limiting min extension from " << ext << " to " << m_minExtension << " ." << std::endl;
      if(ext != 0)
        target *= m_minExtension / ext;
      ret = false;
    }
    if(ext > m_maxExtension) {
      std::cerr << "Limiting max extension from " << ext << " to " << m_maxExtension << " ." << std::endl;
      ext = m_maxExtension;
      if(ext != 0)
        target *= m_maxExtension / ext;
      ret = false;
    }
#endif

    if(!InverseVirtual(target,angles))
      return false;

    // Allow for 4 bar linkage
    float theta = 0;
    if(!Linkage4BarBack(angles[2] ,theta,UseAlternateSolution())) {
      std::cerr << "Linkage failed. \n";
      return false;
    }
    angles[2] = theta - angles[1];

    // Bring angles into a sensible range.
    if(angles[2] > M_PI)
      angles[2] -= M_PI*2;
    if(angles[2] < -M_PI)
      angles[2] += M_PI*2;

    return ret;
  }

  //! Forward kinematics for the leg
  //! Compute the position of the foot relative to the top of the leg from the joint angles.
  bool LegKinematicsC::ForwardDirect(const Eigen::Vector3f &angles,Eigen::Vector3f &at) const
  {
    float theta = angles[2] + angles[1];

    // Theta = Knee Servo
    // Psi = Knee joint
    float psi = Linkage4BarForward(theta,UseAlternateSolution());
    Eigen::Vector3f anglesVirtual = {angles[0],angles[1],psi};

    return ForwardVirtual(anglesVirtual,at);
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
    if(cosAngle > 1.0 || cosAngle < -1.0) {
      //std::cerr << "No angle found for linkage. " << std::endl;
      return false;
    }
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

  //! Compute the maximum extension of the leg
  float LegKinematicsC::MaxExtension() const
  {
    Eigen::Vector3f angles(0,0,0);
    float theta = angles[2] + angles[1];
    float psi = Linkage4BarForward(theta,UseAlternateSolution());
    float y = m_l1 * sin(angles[1]) + m_l2 * sin(angles[1] + psi);
    float z = m_l1 * cos(angles[1]) + m_l2 * cos(angles[1] + psi);
    float dist = sqrt(y * y + z*z) + m_zoff;
    return dist;
  }

  //! Compute the minimum extension of the leg
  float LegKinematicsC::MinExtension() const
  {
    Eigen::Vector3f angles(0,M_PI,0);
    float theta = angles[2] + angles[1];
    float psi = Linkage4BarForward(theta,UseAlternateSolution());
    float y = m_l1 * sin(angles[1]) + m_l2 * sin(angles[1] + psi);
    float z = m_l1 * cos(angles[1]) + m_l2 * cos(angles[1] + psi);
    float dist = sqrt(y * y + z*z) + m_zoff;
    return dist;
  }

  //! Compute the maximum stride length at a given z offset.
  float LegKinematicsC::StrideLength(float zoffset) const
  {
    Eigen::Vector3f anglesIn(0,0,0);

    float theta = anglesIn[2] + anglesIn[1];
    float psi = Linkage4BarForward(theta,UseAlternateSolution());
    Eigen::Vector3f angles = {anglesIn[0],anglesIn[1],psi};

    float y = m_l1 * sin(angles[1]) + m_l2 * sin(angles[1] + angles[2]);
    float z = m_l1 * cos(angles[1]) + m_l2 * cos(angles[1] + angles[2]);
    float r = sqrt(y*y + z*z);

    float rh = zoffset - m_zoff;
    float sqrs = r*r - rh*rh;
    if(sqrs <= 0) {
      return 0;
    }
    return 2*sqrt(sqrs);
  }

}

