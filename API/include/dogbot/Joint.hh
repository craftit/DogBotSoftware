#ifndef DOGBOG_JOINT_HEADER
#define DOGBOG_JOINT_HEADER 1

#include <mutex>
#include <string>
#include <chrono>
#include <jsoncpp/json/json.h>

namespace DogBotN {

  class DogBotAPIC;

  //! Abstract joint.
  //! These give a simplified view of the robot.

  class JointC
  {
  public:
    typedef std::chrono::time_point<std::chrono::steady_clock,std::chrono::duration< double > > TimePointT;

    //! Default constructor
    JointC();

    //! Destructor
    virtual ~JointC();

    //! Access name of device
    std::string Name() const;

    //! Access notes.
    std::string Notes() const;

    //! Set notes.
    void SetNotes(const std::string &notes);

    //! Set name of servo
    void SetName(const std::string &name);

    //! Configure from JSON
    virtual bool ConfigureFromJSON(DogBotAPIC &api,const Json::Value &value);

    //! Get the servo configuration as JSON
    virtual Json::Value ServoConfigAsJSON() const;

    //! Get last reported state of the servo and the time it was taken.
    virtual bool GetState(TimePointT &tick,float &position,float &velocity,float &torque) const;

    //! Estimate state at the given time.
    //! This will linearly extrapolate position, and assume velocity and torque are
    //! the same as the last reading.
    //! If the data is more than 5 ticks away from the
    virtual bool GetStateAt(TimePointT theTime,float &position,float &velocity,float &torque) const;

    //! Update torque for the servo.
    virtual bool DemandTorque(float torque);

    //! Demand a position for the servo
    virtual bool DemandPosition(float position,float torqueLimit);

    //! Last reported position
    float Position() const
    { return m_position; }

    //! Last reported torque
    float Torque() const
    { return m_torque; }

    //! Last reported velocity
    float Velocity() const
    { return m_velocity; }

  protected:
    mutable std::mutex m_mutexJointAdmin;

    std::string m_name;
    std::string m_notes;

    float m_position = 0;
    float m_velocity = 0;
    float m_torque = 0;
  };

}

#endif
