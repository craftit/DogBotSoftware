
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
      float &position,float &velocity,float &torque
  ) const
  {

    position = drivePosition - (refPosition * m_refGain + m_refOffset);
    velocity = driveVelocity - refVelocity;
    torque = driveTorque;
    return true;
  }

  bool Joint4BarLinkageC::Simple2Raw(
       float refPosition,float refTorque,
       float position,float torque,
       float &drivePosition,float &driveTorque
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

    return true;
  }

  //! Get the servo configuration as JSON
  Json::Value Joint4BarLinkageC::ServoConfigAsJSON() const
  {
    Json::Value ret = JointRelativeC::ServoConfigAsJSON();

    return ret;
  }

}
