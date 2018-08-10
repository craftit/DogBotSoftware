
#include "dogbot/LegKinematics.hh"
#include "dogbot/LineABC2d.hh"

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

    //std::cerr << "FV PSI=" << angles[2] << " Pitch=" << angles[1] << std::endl;

    float y = m_l1 * sin(angles[1]) + m_l2 * sin(angles[1] + angles[2]);
    float z = m_l1 * cos(angles[1]) + m_l2 * cos(angles[1] + angles[2]);

    //std::cerr << "FV Y= " << y << " Z=" << z << std::endl;

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

  //! Compute an estimate of the force on a foot and where it is given some angles and torques
  bool LegKinematicsC::ComputeFootForce(
      const Eigen::Vector3f &jointAngles,
      const Eigen::Vector3f &jointVelocity,
      const Eigen::Vector3f &rawTorques,
      Eigen::Vector3f &footAt,
      Eigen::Vector3f &footVelocity,
      Eigen::Vector3f &force
      ) const
  {

    Eigen::Vector3f torque = rawTorques;

    Eigen::Vector3f axisRoll(0,-1,0);
    Eigen::Vector3f axisPitch(-1,0,0);
    Eigen::Vector3f axisKnee(-1,0,0);

    // Compute current position

    double angleRoll = jointAngles[0] * JointDirection(0);
    double anglePitch = jointAngles[1] * JointDirection(1);
    double angleKneeServo = jointAngles[2] * JointDirection(2);

    double velocityRoll = jointVelocity[0] * JointDirection(0);
    double velocityPitch = jointVelocity[1] * JointDirection(1);
    double velocityKneeServo = jointVelocity[2] * JointDirection(2);

    for(int i = 0;i < 3;i++)
      torque[i] *= JointDirection(i);

    //float theta = anglePitch + angleKneeServo;
    float theta = jointAngles[2] + jointAngles[1];

    // Theta = Knee Servo
    // Psi = Knee joint

    float psi = Linkage4BarForward(theta,UseAlternateSolution()) * JointDirection(2);
    float ratio = LinkageSpeedRatio(theta,psi);

    //std::cerr << "CFF PSI=" << psi << " Pitch=" << anglePitch << std::endl;

    float y = m_l1 * sin(anglePitch) + m_l2 * sin(anglePitch + psi);
    float z = m_l1 * cos(anglePitch) + m_l2 * cos(anglePitch + psi);

    //std::cerr << "CFF Y= " << y << " Z=" << z << std::endl;
    // kneeOffset, lowerLegOffset and footOffset are in the hip coordinate system

    Eigen::Vector2f upperLegOffset(
        m_l1 * sin(anglePitch),
        m_l1 * cos(anglePitch)
        );

    Eigen::Vector2f lowerLegOffset(
        m_l2 * sin(anglePitch + psi),
        m_l2 * cos(anglePitch + psi)
        );

    Eigen::Vector2f footOffset = upperLegOffset + lowerLegOffset;

    float footOffsetNorm2 = footOffset.squaredNorm();

    float kneeTorque = -torque[2] * ratio;
    float pitchTorque = torque[1] +torque[2];

    // Pitch force
    Eigen::Vector2f pf = Perpendicular(footOffset) * pitchTorque / footOffsetNorm2;

    // Knee force
    Eigen::Vector2f kf = Perpendicular(lowerLegOffset) * kneeTorque / lowerLegOffset.squaredNorm();

    LineABC2dC line1 = LineABC2dC::CreateFromNormalAndPoint(pf,pf);
    LineABC2dC line2 = LineABC2dC::CreateFromNormalAndPoint(kf,kf);

    Eigen::Vector2f hipf;
    line1.Intersection(line2,hipf);

#if 0
    m_log->info("Torque P:{:+2.2f}  K:{:2.2f}  Pf: {:+2.2f} {:+2.2f}  Kf: {:+2.2f} {:+2.2f}  Hip force  {:+2.2f} {:+2.2f} ",
                pitchTorque,kneeTorque,
                pf[0],pf[1],
                kf[0],kf[1],
                hipf[0],hipf[1]
                             );
#endif

    auto rollRot = Eigen::AngleAxisf(angleRoll,axisRoll);

    Eigen::Vector3f legForce = rollRot * Eigen::Vector3f(0,hipf[0],hipf[1]);

    // Location of centre of knee and pitch servos
    //Eigen::Vector3f hipCentreAt(sin(angleRoll) * zoff,0,cos(angleRoll) * zoff);

    // footAt_fc is in leg coordinates
    const Eigen::Vector3f footAt_fc(
        -sin(angleRoll) * (m_zoff + z), //cos(angles[0]) * xr +
        y,
        -cos(angleRoll) * (m_zoff + z) //sin(angles[0]) * xr +
        );

    // Compute the force from roll on the foot.
    Eigen::Vector3f rollTangent = footAt_fc.cross(axisRoll);

    Eigen::Vector3f rollForce = rollTangent * torque[0] /footAt_fc.squaredNorm();

    // Compute total force in leg coordinates.
    force = rollForce + legForce;

    // Compute the location of the foot in robot coordinates.
    footAt = footAt_fc ;// + m_legOrigin

#if 0
    m_log->info("Torques: {:+2.2f} {:+2.2f} {:+2.2f}  Virt: {:+2.2f} {:+2.2f} {:+2.2f} ({:+1.2})   Leg: {:+2.2f} {:+2.2f} {:+2.2f} ",
                torque[0],torque[1],torque[2],
                torque[0],pitchTorque,kneeTorque,
                ratio,
                force[0],force[1],force[2]);
#endif

    // Compute velocities.

    float kneeVelocity = (velocityKneeServo + velocityPitch) / ratio;
    Eigen::Vector2f kneeVel2d =  Perpendicular(lowerLegOffset) * kneeVelocity ;
    //std::cerr << "Foot " << footOffset[0] << " " << footOffset[1] <<  " KneeVel:" << kneeVel2d[0] << " " << kneeVel2d[1] << std::endl;
    Eigen::Vector2f pitchVel2d = Perpendicular(footOffset) * velocityPitch + kneeVel2d;

    footVelocity = rollRot * Eigen::Vector3f(0,-pitchVel2d[0],pitchVel2d[1]) + rollTangent * velocityRoll;

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

