#ifndef DOGBOG_JOINT_HEADER
#define DOGBOG_JOINT_HEADER 1

#include <mutex>
#include <string>
#include <chrono>
#include <jsoncpp/json/json.h>
#include "dogbot/CallbackArray.hh"
#include "dogbot/protocol.h"

namespace DogBotN {

  class DogBotAPIC;
  class ComsC;

  //! Abstract joint.
  //! These give access for the common functionality needed to control a single joint.

  class JointC
  {
  public:
    typedef std::chrono::time_point<std::chrono::steady_clock,std::chrono::duration< double > > TimePointT;

    typedef std::function<void (TimePointT theTime,double position,double velocity,double torque)> PositionUpdateFuncT;

    typedef std::function<void (ComsParameterIndexT parameter)> ParameterUpdateFuncT;

    //! Default constructor
    JointC();

    //! Destructor
    virtual ~JointC();

    //! Access name of device
    std::string Name() const;

    //! Access notes.
    std::string Notes() const;

    //! Type of joint
    virtual std::string JointType() const;

    //! Set notes.
    void SetNotes(const std::string &notes);

    //! Set name of servo
    void SetName(const std::string &name);

    //! Configure from JSON
    virtual bool ConfigureFromJSON(DogBotAPIC &api,const Json::Value &value);

    //! Get the servo configuration as JSON
    virtual Json::Value ConfigAsJSON() const;

    //! Get last reported state of the servo and the time it was taken.
    virtual bool GetState(TimePointT &tick,double &position,double &velocity,double &torque) const;

    //! Estimate state at the given time.
    //! This will linearly extrapolate position, and assume velocity and torque are
    //! the same as the last reading.
    //! If the data is more than 5 ticks away from the
    virtual bool GetStateAt(TimePointT theTime,double &position,double &velocity,double &torque) const;

    //! Update torque for the servo.
    virtual bool DemandTorque(float torque);

    //! Demand a position for the servo
    virtual bool DemandPosition(float position,float torqueLimit);

    //! Add a update callback for motor position
    CallbackHandleC AddPositionUpdateCallback(const PositionUpdateFuncT &callback)
    { return m_positionCallbacks.Add(callback); }

    //! Add notification callback if a parameter changes.
    CallbackHandleC AddParameterUpdateCallback(const ParameterUpdateFuncT &func)
    { return m_parameterCallbacks.Add(func); }

    //! Last reported position
    float Position() const
    { return m_position; }

    //! Last reported torque
    float Torque() const
    { return m_torque; }

    //! Last reported velocity
    float Velocity() const
    { return m_velocity; }

    //! Is this joint exported ?
    bool IsExported() const
    { return m_export; }

    //! Update coms device
    virtual void UpdateComs(const std::shared_ptr<ComsC> &coms);
  protected:
    CallbackArrayC<PositionUpdateFuncT> m_positionCallbacks;
    CallbackArrayC<ParameterUpdateFuncT> m_parameterCallbacks;

    mutable std::mutex m_mutexJointAdmin;

    std::string m_name;
    std::string m_notes;
    bool m_export = false;
    float m_position = 0; // Radians
    float m_velocity = 0; // Radians per second
    float m_torque = 0;   // Newton meters
  };

}

#endif
