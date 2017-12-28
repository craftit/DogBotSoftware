
#include <string>
#include "dogbot/Joint4BarLinkage.hh"

namespace DogBotN {

  //! Default constructor
  Joint4BarLinkageC::Joint4BarLinkageC()
  {}

  //! Constructor
  Joint4BarLinkageC::Joint4BarLinkageC(std::shared_ptr<JointC> &jointDrive,std::shared_ptr<JointC> &jointRef)
    : JointRelativeC(jointDrive,jointRef)
  {}

  bool Joint4BarLinkageC::Raw2Simple(
      float refPosition,float refVelocity,float refTorque,
      float drivePosition,float driveVelocity,float driveTorque,
      double &position,double &velocity,double &torque
  ) const
  {

    position = drivePosition - (refPosition * m_refGain + m_refOffset);
    velocity = driveVelocity - refVelocity;
    torque = driveTorque;
    return true;
  }

  //! Type of joint
  std::string Joint4BarLinkageC::JointType() const
  { return "4bar"; }


  bool Joint4BarLinkageC::Simple2Raw(
       float refPosition,float refTorque,
       float position,float torque,
       double &drivePosition,double &driveTorque
  ) const
  {
    drivePosition = position + (refPosition * m_refGain + m_refOffset);
    driveTorque = torque;
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
    Json::Value val = value.get("linkage",Json::nullValue);
    if(!val.isNull()) {
      if(!m_legKinematics.ConfigureFromJSON(val))
        return false;
    }
    return true;
  }

  //! Get the servo configuration as JSON
  Json::Value Joint4BarLinkageC::ConfigAsJSON() const
  {
    Json::Value ret = JointRelativeC::ConfigAsJSON();
    ret["linkage"] = m_legKinematics.ConfigAsJSON();
    return ret;
  }

}
