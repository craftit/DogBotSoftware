

#include "dogbot/LegController.hh"
#include "dogbot/Util.hh"
#include "dogbot/LineABC2d.hh"

namespace DogBotN {

  //! Class to manage the positioning of a single leg.

  LegControllerC::LegControllerC(std::shared_ptr<DogBotAPIC> &api,const std::string &legName,bool useVirtualKnee)
   : m_useVirtualKnee(useVirtualKnee),
     m_legName(legName),
     m_api(api)
  {
    Init();
  }

  //! Destructor
  LegControllerC::~LegControllerC()
  {}

  bool LegControllerC::Init()
  {
    m_kinematics = m_api->LegKinematicsByName(m_legName);
    if(!m_kinematics) {
      m_log->error("Failed to find leg '{}' ",m_legName);
      return false;
    }

    m_legJointNames.clear();
    m_legJointNames.push_back(m_legName + "_roll");
    m_legJointNames.push_back(m_legName + "_pitch");
    if(m_useVirtualKnee) {
      m_legJointNames.push_back("virtual_" + m_legName + "_knee");
    } else {
      m_legJointNames.push_back(m_legName + "_knee");
    }

    for(int i = 0;i < 3;i++) {
      m_joints[i] = m_api->GetJointByName(m_legJointNames[i]);
      if(!m_joints[i]) {
        m_log->error("Failed to find {} joint. ",m_legJointNames[i]);
        return false;
      }
    }

    if(m_kinematics)
      m_legOrigin = Eigen::Vector3f(m_kinematics->LegOrigin(0),m_kinematics->LegOrigin(1),m_kinematics->LegOrigin(2));

    return true;
  }

  //! Goto a position in the leg coordinate frame
  //! Returns true if the requested position is reachable
  bool LegControllerC::Goto(float x,float y,float z,float torqueLimit)
  {
    return Goto(Eigen::Vector3f(x,y,z),torqueLimit);
  }

  //! Goto a position
  //! Returns true position is reachable
  bool LegControllerC::Goto(const Eigen::Vector3f &at,float torque)
  {
    Eigen::Vector3f angles;
    if(!m_kinematics) {
      m_log->error("No kinematics for leg {} ",m_legName);
      return false;
    }
    if(!m_useVirtualKnee) {
      if(!m_kinematics->InverseDirect(at,angles)) {
        m_log->warn("Failed to find solution for direct foot location {} {} {} .",at[0],at[1],at[2]);
        return false;
      }
    } else {
      if(!m_kinematics->InverseVirtual(at,angles)) {
        m_log->warn("Failed to find solution for virtual foot location {} {} {}.",at[0],at[1],at[2]);
        return false;
      }
    }

#if 0
    m_log->info("Setting angles to {} {} {}  for  {} {} {}  UseVirt:{} ",
                   DogBotN::Rad2Deg(angles[0]),DogBotN::Rad2Deg(angles[1]),DogBotN::Rad2Deg(angles[2]),
                   at[0],at[1],at[2],m_useVirtualKnee);
#endif


    // FIXME:- If move fails what should we do ?

    return GotoJointAngles(angles,torque);
  }

  //! Goto a joint angles
  bool LegControllerC::GotoJointAngles(const Eigen::Vector3f &angles,float torque)
  {
    bool ret = true;
    if(!m_joints[0]->DemandPosition(angles[0],torque))
      ret = false;
    if(!m_joints[1]->DemandPosition(angles[1],torque))
      ret = false;
    if(!m_joints[2]->DemandPosition(angles[2],torque))
      ret = false;
    return ret;
  }

  //! Goto a joint angles
  bool LegControllerC::GotoJointAngles(float roll,float pitch,float knee,float torqueLimit)
  {
    return GotoJointAngles(Eigen::Vector3f(roll,pitch,knee),torqueLimit);
  }

  //! Get current joint angles
  bool LegControllerC::GetJointAngles(TimePointT theTime,Eigen::Vector3f &angles)
  {
    double velocity = 0,torque = 0;
    double rolld = 0,pitchd = 0,kneed = 0;
    bool ret = true;
    if(!m_joints[0]->GetStateAt(theTime,rolld,velocity,torque))
      ret = false;
    if(!m_joints[1]->GetStateAt(theTime,pitchd,velocity,torque))
      ret = false;
    if(!m_joints[2]->GetStateAt(theTime,kneed,velocity,torque))
      ret = false;
    angles[0] = rolld;
    angles[1] = pitchd;
    angles[2] = kneed;
    return ret;
  }

  //! Get current joint angles
  bool LegControllerC::GetJointAngles(double theTime,float &roll,float &pitch,float &knee)
  {
    Eigen::Vector3f angles;
    DogBotN::TimePointT::duration timeSinceEpoch(theTime);
    DogBotN::TimePointT atTimePoint(timeSinceEpoch);
    if(!GetJointAngles(atTimePoint,angles))
      return false;
    roll  = angles[0];
    pitch = angles[1];
    knee  = angles[2];
    return true;
  }

  //! Get current joint states
  bool LegControllerC::GetJointStates(
      double theTime,
      float &angleRoll,float &anglePitch,float &angleKnee,
      float &velocityRoll,float &velocityPitch,float &velocityKnee,
      float &torqueRoll,float &torquePitch,float &torqueKnee
  )
  {
    DogBotN::TimePointT::duration timeSinceEpoch(theTime);
    DogBotN::TimePointT atTimePoint(timeSinceEpoch);

    bool ret = true;
    {
      double angle= 0,velocity = 0,torque = 0;
      if(!m_joints[0]->GetStateAt(atTimePoint,angle,velocity,torque))
        ret = false;
      angleRoll = angle;
      velocityRoll = velocity;
      torqueRoll = torque;
    }
    {
      double angle= 0,velocity = 0,torque = 0;
      if(!m_joints[1]->GetStateAt(atTimePoint,angle,velocity,torque))
        ret = false;
      anglePitch = angle;
      velocityPitch = velocity;
      torquePitch = torque;
    }
    {
      double angle= 0,velocity = 0,torque = 0;
      if(!m_joints[2]->GetStateAt(atTimePoint,angle,velocity,torque))
        ret = false;
      angleKnee = angle;
      velocityKnee = velocity;
      torqueKnee = torque;
    }
    return ret;
  }

  //! Compute the force on a foot and where it is.
  bool LegControllerC::ComputeFootForce(const TimePointT &atTime,Eigen::Vector3f &footAt,Eigen::Vector3f &force)
  {
    if(m_kinematics)
      m_legOrigin = Eigen::Vector3f(m_kinematics->LegOrigin(0),m_kinematics->LegOrigin(1),m_kinematics->LegOrigin(2));

    Eigen::Vector3f axisRoll(0,-1,0);
    Eigen::Vector3f axisPitch(-1,0,0);
    Eigen::Vector3f axisKnee(-1,0,0);


    // Get current state,  Roll, Pitch, Knee

    double position[3];
    double velocity[3];
    double torque[3];

    for(int i = 0;i < 3;i++)
      m_joints[i]->GetStateAt(atTime,position[i],velocity[i],torque[i]);

    // Compute current position

    double angleRoll = position[0] * m_kinematics->JointDirection(0);
    double anglePitch = position[1] * m_kinematics->JointDirection(1);
    double angleKneeServo = position[2] * m_kinematics->JointDirection(2);

    for(int i = 0;i < 3;i++)
      torque[i] *= m_kinematics->JointDirection(i);

    float theta = anglePitch - angleKneeServo;
    float l1 = m_kinematics->LengthUpperLeg();
    float l2 = m_kinematics->LengthLowerLeg();
    float zoff = m_kinematics->LengthZDrop();

    // Theta = Knee Servo
    // Psi = Knee joint

    float psi = m_kinematics->Linkage4BarForward(theta,m_kinematics->UseAlternateSolution()) * m_kinematics->JointDirection(2);
    float ratio = m_kinematics->LinkageSpeedRatio(theta,psi);

    float y = l1 * sin(anglePitch) + l2 * sin(anglePitch + psi);
    float z = l1 * cos(anglePitch) + l2 * cos(anglePitch + psi);

    // kneeOffset, lowerLegOffset and footOffset are in the hip coordinate system

    Eigen::Vector2f upperLegOffset(
        l1 * sin(anglePitch),
        l1 * cos(anglePitch)
        );

    Eigen::Vector2f lowerLegOffset(
        l2 * sin(anglePitch + psi),
        l2 * cos(anglePitch + psi)
        );

    Eigen::Vector2f footOffset = upperLegOffset + lowerLegOffset;

    float footOffsetNorm2 = footOffset.squaredNorm();
    float footOffsetNorm = sqrt(footOffsetNorm2);

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

    // footAt_fc is in foot coordinates
    Eigen::Vector3f footAt_fc(
        sin(angleRoll) * (zoff + z), //cos(angles[0]) * xr +
        y,
        cos(angleRoll) * (zoff + z) //sin(angles[0]) * xr +
        );

    // Compute the force from roll on the foot.
    Eigen::Vector3f rollForce = (footAt_fc.cross(axisRoll) * torque[0] )/(footAt_fc.squaredNorm());

    // Compute total force in leg coordinates.
    Eigen::Vector3f force_fc = rollForce + legForce;

    // Compute force in robot coordinates.
    force = Eigen::Vector3f(-force_fc[0],force_fc[1],-force_fc[2]);

    // Compute the location of the foot in robot coordinates.
    footAt = m_legOrigin + Eigen::Vector3f(-footAt_fc[0],footAt_fc[1],-footAt_fc[2]);

#if 0
    m_log->info("Torques: {:+2.2f} {:+2.2f} {:+2.2f}  Virt: {:+2.2f} {:+2.2f} {:+2.2f} ({:+1.2})   Leg: {:+2.2f} {:+2.2f} {:+2.2f} ",
                torque[0],torque[1],torque[2],
                torque[0],pitchTorque,kneeTorque,
                ratio,
                force[0],force[1],force[2]);
#endif

    return true;
  }

  //! Compute an estimate of the force on a foot and where it is.
  bool LegControllerC::ComputeFootForce(
      double atTime,
      float &positionX,float &positionY,float &positionZ,
      float &forceX,float &forceY,float &forceZ)
  {
    DogBotN::TimePointT::duration timeSinceEpoch(atTime);
    DogBotN::TimePointT atTimePoint(timeSinceEpoch);

    Eigen::Vector3f footAt;
    Eigen::Vector3f force;
    if(!ComputeFootForce(atTimePoint,footAt,force))
      return false;
    positionX = footAt[0];
    positionY = footAt[1];
    positionZ = footAt[2];
    forceX = force[0];
    forceY = force[1];
    forceZ = force[2];
    return true;
  }


}


