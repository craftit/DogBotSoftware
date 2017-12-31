
#include "dogbot/Joint.hh"
#include <string>

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

    return true;
  }

  //! Get the servo configuration as JSON
  Json::Value JointC::ConfigAsJSON() const
  {
    Json::Value  ret;

    std::lock_guard<std::mutex> lock(m_mutexJointAdmin);
    ret["name"] = m_name;
    ret["notes"] = m_notes;
    ret["type"] = JointType();

    return ret;
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
    return false;
  }

  //! Update coms device
  void JointC::UpdateComs(const std::shared_ptr<ComsC> &coms)
  {
  }


}
