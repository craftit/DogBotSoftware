
#include "dogbot/Joint.hh"
#include "dogbot/Util.hh"
#include <string>
#include <iostream>
#include <math.h>

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
    m_notes = conf.get("notes","").asString();
    m_serialNumber = conf.get("serial_number",m_serialNumber).asString();
    return true;
  }

  //! Get the servo configuration as JSON
  Json::Value JointC::ConfigAsJSON() const
  {
    Json::Value  ret;

    std::lock_guard<std::mutex> lock(m_mutexJointAdmin);
    ret["name"] = m_name;
    ret["notes"] = m_notes;
    ret["serial_number"] = m_serialNumber;
    ret["type"] = JointType();
    return ret;
  }

  //! Access name of device
  std::string JointC::Name() const
  {
    std::lock_guard<std::mutex> lock(m_mutexJointAdmin);
    return m_name;
  }

  //! Access serial number if set.
  std::string JointC::SerialNumber() const
  {
    std::lock_guard<std::mutex> lock(m_mutexJointAdmin);
    return m_serialNumber;
  }

  //! Set name of servo
  void JointC::SetName(const std::string &name)
  {
    std::lock_guard<std::mutex> lock(m_mutexJointAdmin);
    m_name = name;

  }

  //! Access notes.
  std::string JointC::Notes() const
  {
    std::lock_guard<std::mutex> lock(m_mutexJointAdmin);
    return m_notes;
  }

  //! Set notes.
  void JointC::SetNotes(const std::string &notes)
  {
    std::lock_guard<std::mutex> lock(m_mutexJointAdmin);
    m_notes = notes;
  }


  //! Get last reported state of the servo and the time it was taken.
  bool JointC::GetState(TimePointT &tick,double &position,double &velocity,double &torque) const
  {
    return false;
  }

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
    if(isnanf(position) || isnanf(torqueLimit))
      return false;
    position = m_demandPosition;
    torqueLimit = m_demandTorqueLimit;
    return true;
  }

  //! Update coms device
  void JointC::UpdateComs(const std::shared_ptr<ComsC> &coms)
  {
  }

  //! Add a update callback for motor position
  CallbackHandleC JointC::AddPositionUpdateCallback(const PositionUpdateFuncT &callback)
  { return m_positionCallbacks.Add(callback); }

  //! Add a callback when the motor target position is updated
  CallbackHandleC JointC::AddDemandUpdateCallback(const DemandUpdateFuncT &callback)
  { return m_demandCallbacks.Add(callback); }


  //! Add notification callback if a parameter changes.
  CallbackHandleC JointC::AddParameterUpdateCallback(const ParameterUpdateFuncT &func)
  { return m_parameterCallbacks.Add(func); }


  //! Move to position and wait until it gets there or stalls.
  JointMoveStatusT JointC::MoveWait(float targetPosition,float torqueLimit,double timeOut)
  {
    std::timed_mutex done;
    done.lock();

    TimePointT startTime = TimePointT::clock::now();
    if(!DemandPosition(targetPosition,torqueLimit)) {
      m_log->info("Failed to set {} to demand position {} . ",Name(),targetPosition);
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
            m_log->info("Got {} to position {} . ",Name(),targetPosition);
            ret = JMS_Done;
            done.unlock();
            return ;
          }
          // Stalled ?
          double timeSinceStart =  (theTime - startTime).count();
          if(fabs(velocity) < (M_PI/64.0) && torque >= (torqueLimit * 0.95) && timeSinceStart > 0.5){
            m_log->info("Stalled {} at {}. ",Name(),position);

            ret = JMS_Stalled;
            done.unlock();
            return ;
          }

        }
    );
    using Ms = std::chrono::milliseconds;

    if(!done.try_lock_for(Ms((int) (1000 * timeOut)))) {
      m_log->info("Timeout {} going to {}. ",Name(),targetPosition);
      cb.Remove();
      return JMS_TimeOut;
    }
    cb.Remove();
    return ret;
  }



}
