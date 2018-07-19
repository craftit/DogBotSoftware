
#include "dogbot/Joint.hh"
#include "dogbot/Util.hh"
#include <string>
#include <iostream>
#include <cmath>

namespace DogBotN {

  //! Default constructor
  JointC::JointC()
  {}

  //! Destructor
  JointC::~JointC()
  {}


  //! Type of joint
  std::string JointC::JointType() const
  {
    return "joint";
  }

  //! Configure from JSON
  bool JointC::ConfigureFromJSON(DogBotAPIC &api,const Json::Value &conf)
  {
    std::lock_guard<std::mutex> lock(m_mutexJointAdmin);
    m_name = conf.get("name","?").asString();
    // m_notes = conf.get("notes","").asString();
    // m_serialNumber = conf.get("serial_number",m_serialNumber).asString();
    return true;
  }
  //! Get the servo configuration as JSON
  void JointC::ConfigAsJSON(Json::Value &ret) const
  {
    std::lock_guard<std::mutex> lock(m_mutexJointAdmin);
    ret["name"] = m_name;
    //ret["notes"] = m_notes;
    //ret["serial_number"] = m_serialNumber;
    ret["type"] = JointType();
  }

  //! Access name of device
  std::string JointC::Name() const
  {
    std::lock_guard<std::mutex> lock(m_mutexJointAdmin);
    return m_name;
  }

  //! Set name of servo
  void JointC::SetName(const std::string &name)
  {
    std::lock_guard<std::mutex> lock(m_mutexJointAdmin);
    m_name = name;
  }


  //! Get last reported state of the servo and the time it was taken.
  bool JointC::GetState(TimePointT &tick,double &position,double &velocity,double &torque) const
  {
    return false;
  }

  //! Get last reported state of the servo and the time it was taken in seconds.
  bool JointC::GetState(double &theTime,double &position,double &velocity,double &torque) const
  {
    TimePointT tick;
    if(!GetState(tick,position,velocity,torque))
      return false;
    theTime = tick.time_since_epoch().count();
    return true;
  }

  //! Estimate state at the given time.
  //! This will linearly extrapolate position, and assume velocity and torque are
  //! the same as the last reading.
  //! If the data is more than 5 ticks away from the current time the method returns false.
  bool JointC::GetStateAt(double theTime,double &position,double &velocity,double &torque) const
  {
    TimePointT::duration timeSinceEpoch(theTime);
    return GetStateAt(TimePointT(timeSinceEpoch),position,velocity,torque);
  }

  //! Access update tick duration
  double JointC::TickDuration() const
  { return -1; }

  //! Estimate state at the given time.
  //! This will linearly extrapolate position, and assume velocity and torque are
  //! the same as the last reading.
  //! If the data is more than 5 ticks away from the
  bool JointC::GetStateAt(TimePointT theTime,double &position,double &velocity,double &torque) const
  {
    return false;
  }

  //! Update torque for the servo.
  bool JointC::DemandTorque(float torque)
  {
    return false;
  }

  //! Demand a position for the servo
  bool JointC::DemandPosition(float position,float torqueLimit)
  {
    m_demandPosition = position;
    m_demandTorqueLimit = torqueLimit;
    for(auto &a : m_demandCallbacks.Calls()) {
      if(a) a(position,torqueLimit);
    }
    return true;
  }

  //! Get current demand
  bool JointC::GetDemand(double &position,double &torqueLimit)
  {
    if(std::isnan(m_demandPosition) || std::isnan(m_demandTorqueLimit))
      return false;
    position = m_demandPosition;
    torqueLimit = m_demandTorqueLimit;
    return true;
  }

  //! Get current demand
  bool JointC::GetDemandTrajectory(double &position,double &torque)
  {
    if(std::isnan(m_demandPosition) || std::isnan(m_demandTorque))
      return false;
    position = m_demandPosition;
    torque = m_demandTorque;
    return true;
  }

  //! Set the trajectory
  bool JointC::SetupTrajectory(float period,float torqueLimit)
  {
    m_demandTorqueLimit = torqueLimit;
    return true;
  }

  //! Demand a next position in a trajectory
  bool JointC::DemandTrajectory(float position,float torque)
  {
    m_demandPosition = position;
    m_demandTorque = torque;
#if 0
    for(auto &a : m_demandCallbacks.Calls()) {
      if(a) a(position,m_demandTorqueLimit);
    }
#endif
    return true;
  }

  //! Add a update callback for motor position
  CallbackHandleC JointC::AddPositionUpdateCallback(const PositionUpdateFuncT &callback)
  { return m_positionCallbacks.Add(callback); }

  //! Add a callback when the motor target position is updated
  CallbackHandleC JointC::AddDemandUpdateCallback(const DemandUpdateFuncT &callback)
  { return m_demandCallbacks.Add(callback); }


  //! Move to position and wait until it gets there or stalls.
  JointMoveStatusT JointC::MoveWait(float targetPosition,float torqueLimit,double timeOut)
  {
    std::timed_mutex done;
    done.lock();

    TimePointT startTime = TimePointT::clock::now();
    if(!DemandPosition(targetPosition,torqueLimit)) {
      m_logJoint->info("Failed to set {} to demand position {} . ",Name(),targetPosition);
      return JMS_Error;
    }

    float tolerance = Deg2Rad(2.0);

    JointMoveStatusT ret = JMS_Error;

    CallbackHandleC cb = AddPositionUpdateCallback(
        [this,targetPosition,torqueLimit,&ret,&done,&startTime,tolerance]
         (TimePointT theTime,double position,double velocity,double torque)
        {
          // At target position ?
          if(fabs(position - targetPosition) < tolerance) {
            m_logJoint->info("Got {} to position {} . ",Name(),targetPosition);
            ret = JMS_Done;
            done.unlock();
            return ;
          }
          // Stalled ?
          double timeSinceStart =  (theTime - startTime).count();
          if(fabs(velocity) < (M_PI/64.0) && torque >= (torqueLimit * 0.95) && timeSinceStart > 0.5){
            m_logJoint->info("Stalled {} at {}. ",Name(),position);

            ret = JMS_Stalled;
            done.unlock();
            return ;
          }

        }
    );
    using Ms = std::chrono::milliseconds;

    if(!done.try_lock_for(Ms((int) (1000 * timeOut)))) {
      m_logJoint->info("Timeout {} going to {}. ",Name(),targetPosition);
      cb.Remove();
      return JMS_TimeOut;
    }
    cb.Remove();
    return ret;
  }



}
