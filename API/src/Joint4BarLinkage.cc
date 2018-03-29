
#include <string>
#include "dogbot/Joint4BarLinkage.hh"
#include "dogbot/DogBotAPI.hh"

namespace DogBotN {

  //! Default constructor
  Joint4BarLinkageC::Joint4BarLinkageC()
  {}

  //! Constructor
  Joint4BarLinkageC::Joint4BarLinkageC(std::shared_ptr<JointC> &jointDrive,std::shared_ptr<JointC> &jointRef)
    : JointRelativeC(jointDrive,jointRef)
  {}

  //! Type of joint
  std::string Joint4BarLinkageC::JointType() const
  { return "4bar"; }

  bool Joint4BarLinkageC::Raw2Simple(
      float refPosition,float refVelocity,float refTorque,
      float drivePosition,float driveVelocity,float driveTorque,
      double &position,double &velocity,double &torque
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
       float refPosition,float refTorque,
       float position,float torque,
       double &drivePosition,double &driveTorque
  ) const
  {
    float theta;
    if(!m_legKinematics->Linkage4BarBack(position,theta,m_legKinematics->UseAlternateSolution())) {
      m_log->error("No solution for angle {} for joint {} ",position,Name());
      return false;
    }
    float ratio = m_legKinematics->LinkageSpeedRatio(theta,position);

    drivePosition = theta + (refPosition * m_refGain + m_refOffset);
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
    return true;
  }

  //! Get the servo configuration as JSON
  Json::Value Joint4BarLinkageC::ConfigAsJSON() const
  {
    Json::Value ret = JointRelativeC::ConfigAsJSON();
    if(m_legKinematics)
      ret["legKinematics"] = m_legKinematics->Name();
    return ret;
  }

}
