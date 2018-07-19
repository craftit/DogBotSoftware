#ifndef DOGBOG_JOINT_HEADER
#define DOGBOG_JOINT_HEADER 1

#include <mutex>
#include <string>
#include <chrono>
#include <jsoncpp/json/json.h>
#include "dogbot/CallbackArray.hh"
#include "dogbot/protocol.h"
#include "dogbot/Common.hh"
#include <spdlog/spdlog.h>

namespace DogBotN {

  enum JointMoveStatusT { JMS_Done, JMS_Stalled, JMS_TimeOut, JMS_IncorrectMode, JMS_Error };

  //! Abstract joint.
  //! These give access for the common functionality needed to control a single joint.

  class JointC
  {
  public:

    typedef std::function<void (TimePointT theTime,double position,double velocity,double torque)> PositionUpdateFuncT;

    typedef std::function<void (double position,double torque)> DemandUpdateFuncT;

    typedef std::function<void (ComsParameterIndexT parameter)> ParameterUpdateFuncT;

    //! Default constructor
    JointC();

    //! Destructor
    virtual ~JointC();

    //! Access name of device
    std::string Name() const;

    //! Set name of servo
    virtual void SetName(const std::string &name);

    //! Type of joint
    virtual std::string JointType() const;

    //! Configure from JSON
    virtual bool ConfigureFromJSON(DogBotAPIC &api,const Json::Value &value);

    //! Get the servo configuration as JSON
    virtual void ConfigAsJSON(Json::Value &value) const;

    //! Get last reported state of the servo and the time it was taken.
    virtual bool GetState(TimePointT &tick,double &position,double &velocity,double &torque) const;

    //! Estimate state at the given time.
    //! This will linearly extrapolate position, and assume velocity and torque are
    //! the same as the last reading.
    //! If the data is more than 5 ticks away from the current time the method returns false.
    virtual bool GetStateAt(TimePointT theTime,double &position,double &velocity,double &torque) const;

    //! Get last reported state of the servo and the time it was taken in seconds.
    bool GetState(double &theTime,double &position,double &velocity,double &torque) const;

    //! Estimate state at the given time.
    //! This will linearly extrapolate position, and assume velocity and torque are
    //! the same as the last reading.
    //! If the data is more than 5 ticks (See TickDuration) away from the current time the method returns false.
    bool GetStateAt(double theTime,double &position,double &velocity,double &torque) const;

    //! Access update tick duration
    //! This is the expected time between joint updates.
    //! This is set to -1 if the duration is not yet known.
    virtual double TickDuration() const;

    //! Update torque for the servo.
    virtual bool DemandTorque(float torque);

    //! Demand a position for the servo, torque limit in Newton-meters
    virtual bool DemandPosition(float position,float torqueLimit);

    //! Set the trajectory
    //! update period in seconds, torque limit in Newton-meters
    virtual bool SetupTrajectory(float updatePeriod,float torqueLimit);

    //! Demand a next position in a trajectory
    //! position in radians,
    //! torque in Newton-meters
    virtual bool DemandTrajectory(float position,float torque = 0);

    //! Add a update callback for motor position
    virtual CallbackHandleC AddPositionUpdateCallback(const PositionUpdateFuncT &callback);

    //! Add a callback when the motor target position is updated
    virtual CallbackHandleC AddDemandUpdateCallback(const DemandUpdateFuncT &callback);

    //! Get current demand
    virtual bool GetDemand(double &position,double &torqueLimit);

    //! Get current demand
    virtual bool GetDemandTrajectory(double &position,double &torque);

    //! Last reported position
    float Position() const
    { return m_position; }

    //! Last reported torque
    float Torque() const
    { return m_torque; }

    //! Last reported velocity
    float Velocity() const
    { return m_velocity; }

    //! Move to position and wait until it gets there or stalls.
    //! timeout is in seconds.
    JointMoveStatusT MoveWait(float position,float torqueLimit,double timeOut = 3.0);

  protected:
    CallbackArrayC<PositionUpdateFuncT> m_positionCallbacks;
    CallbackArrayC<DemandUpdateFuncT> m_demandCallbacks;

    mutable std::mutex m_mutexJointAdmin;

    std::shared_ptr<spdlog::logger> m_logJoint = spdlog::get("console");

    std::string m_name;

    float m_demandPosition = nan("");
    float m_demandTorqueLimit = nan("");
    float m_demandTorque = nan("");

    float m_position = 0; // Radians
    float m_velocity = 0; // Radians per second
    float m_torque = 0;   // Newton meters
  };

}

#endif
