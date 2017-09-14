#ifndef DOGBOG_DOGBOTAPI_HEADER
#define DOGBOG_DOGBOTAPI_HEADER 1

#include "dogbot/Servo.hh"
#include <json/json.h>
#include "SerialComs.hh"

namespace DogBotN {

  //! Dogbot device control

  //! This does low level management of the robot, configuration of the drivers and status monitoring.

  class DogBotAPIC
  {
  public:
    //! Constructor
    DogBotAPIC(const std::string &configFile = "");

    //! Set the logger to use
    void SetLogger(std::shared_ptr<spdlog::logger> &log);

    //! Load configuration for the robot.
    bool Init(const std::string &configFile);

    //! Shutdown controller.
    bool Shutdown();

  protected:
    enum DriverStateT {
      DS_Init,
      DS_NoConnection,
      DS_Connected,
      DS_Calibrated,
      DS_Error
    } m_driverState = DS_Init;

    //! Monitor thread
    void RunMonitor();

    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");

    std::string m_deviceName;
    Json::Value m_configRoot;
    SerialComsC m_coms;
    std::vector<ServoC> m_servos;

    std::thread m_threadMonitor;

    bool m_started = false;
    bool m_terminate = false;
  };


}

#endif
