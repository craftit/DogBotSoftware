

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
  // The vector has the angles  indexed in the following order : roll,pitch,knee
  bool LegControllerC::GetJointStates(TimePointT theTime,Eigen::Vector3f &angles,Eigen::Vector3f &velocity,Eigen::Vector3f &torque)
  {
    bool ret = true;
    for(int i = 0;i < 3;i++) {
      double t = 0,v = 0,d = 0;
      if(!m_joints[i]->GetStateAt(theTime,d,v,t))
        ret = false;
      angles[i] = d;
      velocity[i] = v;
      torque[i] = t;
    }
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

  //! Compute an estimate of the force state
  bool LegControllerC::ComputeFootState(const DogBotN::TimePointT &atTime,Eigen::Vector3f &footAt,Eigen::Vector3f &footVel,Eigen::Vector3f &force)
  {
    // Get current state,  Roll, Pitch, Knee
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f torque;

    if(!GetJointStates(atTime,position,velocity,torque))
      return false;

    return m_kinematics->ComputeFootForce(position,velocity,torque,footAt,footVel,force);
  }

  //! Compute the force on a foot and where it is.
  bool LegControllerC::ComputeFootForce(const TimePointT &atTime,Eigen::Vector3f &footAt,Eigen::Vector3f &force)
  {
    // Get current state,  Roll, Pitch, Knee
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f torque;

    if(!GetJointStates(atTime,position,velocity,torque))
      return false;

    Eigen::Vector3f footVel;

    return m_kinematics->ComputeFootForce(position,velocity,torque,footAt,footVel,force);
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


