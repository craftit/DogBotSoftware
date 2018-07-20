
#include <string>
#include "dogbot/Joint4BarLinkage.hh"
#include "dogbot/DogBotAPI.hh"
#include "dogbot/Util.hh"

#define DODEBUG 0
#if DODEBUG
#define ONDEBUG(x) x
#else
#define ONDEBUG(x)
#endif

namespace DogBotN {

  //! Default constructor
  Joint4BarLinkageC::Joint4BarLinkageC()
  {
  }

  //! Constructor
  Joint4BarLinkageC::Joint4BarLinkageC(std::shared_ptr<JointC> &jointDrive,std::shared_ptr<JointC> &jointRef)
    : JointRelativeC(jointDrive,jointRef)
  {
  }

  //! Type of joint
  std::string Joint4BarLinkageC::JointType() const
  { return "4bar"; }

  //! Initialise joint
  void Joint4BarLinkageC::Init()
  {
    if(m_legKinematics) {
      m_minAngle = m_legKinematics->Linkage4BarForward(0,m_legKinematics->UseAlternateSolution());
      m_maxAngle = m_legKinematics->Linkage4BarForward(M_PI,m_legKinematics->UseAlternateSolution());
      if(m_maxAngle < m_minAngle) {
        std::swap(m_maxAngle,m_minAngle);
      }
      ONDEBUG(std::cerr << "Joint " << Name() << " Angle range: " << Rad2Deg(m_minAngle) << " " << Rad2Deg(m_maxAngle) << std::endl);
    }
  }

  bool Joint4BarLinkageC::Raw2Simple(
      float refPosition,
      float refVelocity,
      float refTorque,
      float drivePosition,
      float driveVelocity,
      float driveTorque,
      double &position,
      double &velocity,
      double &torque
  ) const
  {
    float theta = drivePosition - (refPosition * m_refGain + m_refOffset);

    // Theta = Knee Servo
    // Psi = Knee joint
    float psi = m_legKinematics->Linkage4BarForward(theta,m_legKinematics->UseAlternateSolution());
    float ratio = m_legKinematics->LinkageSpeedRatio(theta,psi);

    position = psi;
    velocity = (driveVelocity - refVelocity * m_refGain) / ratio;
    // FIXME:- This and its inverse really needs checking!
    torque = (driveTorque - refTorque / m_refGain) * ratio;
    return true;
  }

  bool Joint4BarLinkageC::Simple2Raw(
       float refPosition,
       float refTorque,
       float position,
       float torque,
       double &drivePosition,
       double &driveTorque
  ) const
  {
    float theta;
    if(position < m_minAngle || position > m_maxAngle) {
      m_logJoint->error("Requested joint angle {} out of valid range {} to {} for joint {} ",position,m_minAngle,m_maxAngle,Name());
      return false;
    }

    if(!m_legKinematics->Linkage4BarBack(position,theta,m_legKinematics->UseAlternateSolution())) {
      m_logJoint->error("No solution for angle {} for joint {} based on torque {}; reference {} and {}; drive {} and {} ",position,Name(),torque,refPosition,refTorque,drivePosition,driveTorque);
      return false;
    }
    float ratio = m_legKinematics->LinkageSpeedRatio(theta,position);

    drivePosition = theta + (refPosition * m_refGain + m_refOffset);
    if(drivePosition < -M_PI) drivePosition += 2 * M_PI;
    if(drivePosition > M_PI) drivePosition -= 2 * M_PI;
    driveTorque = torque / ratio + refTorque/m_refGain;
    return true;
  }

  //! Configure from JSON
  bool Joint4BarLinkageC::ConfigureFromJSON(
      DogBotAPIC &api,
      const Json::Value &value
      )
  {
    if(!JointRelativeC::ConfigureFromJSON(api,value))
      return false;
    std::string legName = value.get("legKinematics","default").asString();
    m_legKinematics = api.LegKinematicsByName(legName);
    if(!m_legKinematics) {
      api.Log().error("Failed to configure leg kinematics. '{}' ",legName);
      return false;
    }
    Init();
    return true;
  }

  //! Get the servo configuration as JSON
  void Joint4BarLinkageC::ConfigAsJSON(Json::Value &value) const
  {
    JointRelativeC::ConfigAsJSON(value);
    if(m_legKinematics)
      value["legKinematics"] = m_legKinematics->Name();
  }

}
