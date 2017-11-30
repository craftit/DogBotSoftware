
#include <string>
#include "../include/dogbot/JointRelative.hh"

namespace DogBotN {

  //! Default constructor
  JointRelativeC::JointRelativeC()
  {}

  //! Constructor
  JointRelativeC::JointRelativeC(std::shared_ptr<JointC> &jointDrive,std::shared_ptr<JointC> &jointRef)
    : m_jointDrive(jointDrive),
      m_jointRef(jointRef)
  {}

  bool JointRelativeC::Raw2Simple(
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

  bool JointRelativeC::Simple2Raw(
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
  bool JointRelativeC::ConfigureFromJSON(
      DogBotAPIC &api,
      const Json::Value &value
      )
  {
    if(!JointC::ConfigureFromJSON(api,value))
      return false;

    return true;
  }

  //! Get the servo configuration as JSON
  Json::Value JointRelativeC::ServoConfigAsJSON() const
  {
    Json::Value ret = JointC::ServoConfigAsJSON();

    return ret;
  }

  //! Get last reported state of the servo and the time it was taken.
  bool JointRelativeC::GetState(TimePointT &tick,float &position,float &velocity,float &torque) const
  {
    float drivePosition = 0;
    float driveVelocity = 0;
    float driveTorque = 0;

    if(!m_jointDrive->GetState(tick,drivePosition,driveVelocity,driveTorque))
      return false;

    float refPosition = 0;
    float refVelocity = 0;
    float refTorque = 0;

    if(!m_jointRef->GetStateAt(tick,refPosition,refVelocity,refTorque))
      return false;

    Raw2Simple(refPosition,refVelocity,refTorque,
               drivePosition,driveVelocity,driveTorque,
               position,velocity,torque
               );

    return true;
  }

  //! Estimate state at the given time.
  //! This will linearly extrapolate position, and assume velocity and torque are
  //! the same as the last reading.
  //! If the data is more than 5 ticks away from the
  bool JointRelativeC::GetStateAt(TimePointT tick,float &position,float &velocity,float &torque) const
  {
    float drivePosition = 0;
    float driveVelocity = 0;
    float driveTorque = 0;

    if(!m_jointDrive->GetState(tick,drivePosition,driveVelocity,driveTorque)) {
      return false;
    }

    float refPosition = 0;
    float refVelocity = 0;
    float refTorque = 0;

    if(!m_jointRef->GetStateAt(tick,refPosition,refVelocity,refTorque)) {
      return false;
    }

    return Raw2Simple(refPosition,refVelocity,refTorque,
               drivePosition,driveVelocity,driveTorque,
               position,velocity,torque
               );
  }

  //! Update torque for the servo.
  bool JointRelativeC::DemandTorque(float torque)
  {
    return m_jointDrive->DemandTorque(torque);
  }

  //! Demand a position for the servo
  bool JointRelativeC::DemandPosition(float position,float torqueLimit)
  {
    m_demandPosition = position;
    m_demandTorqueLimit = torqueLimit;

    float refPosition = 0;
    float refVelocity = 0;
    float refTorque = 0;

    TimePointT tick = TimePointT::clock::now();
    if(!m_jointRef->GetStateAt(tick,refPosition,refVelocity,refTorque)) {
      return false;
    }

    float drivePosition = 0;
    float driveTorqueLimit = 0;
    if(!Simple2Raw(
        refPosition,refTorque,
        position,torqueLimit,
        drivePosition,driveTorqueLimit)) {
      return false;
    }

    m_demandPosition = position;
    m_demandTorqueLimit = torqueLimit;

    m_jointDrive->DemandPosition(drivePosition,driveTorqueLimit);

    return true;
  }

}
