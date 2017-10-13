#ifndef DOGBOG_DOGBOTAPI_HEADER
#define DOGBOG_DOGBOTAPI_HEADER 1

#include "dogbot/Servo.hh"
#include <json/json.h>
#include "dogbot/SerialComs.hh"

namespace DogBotN {

  //! Dogbot device control

  //! This does low level management of the robot, configuration of the drivers and status monitoring.

  class DogBotAPIC
  {
  public:
    //! Constructor
    DogBotAPIC(const std::string &configFile = "");

    //! Construct with coms object
    DogBotAPIC(const std::shared_ptr<SerialComsC> &coms);

    //! Connect to coms object.
    bool Connect(const std::shared_ptr<SerialComsC> &coms);

    //! Set the logger to use
    void SetLogger(std::shared_ptr<spdlog::logger> &log);

    //! Load configuration for the robot.
    bool Init(const std::string &configFile);

    //! Read calibration from a device.
    bool ReadCalibration(int deviceId,MotorCalibrationC &cal);

    //! Write calibration to a device.
    bool WriteCalibration(int deviceId,const MotorCalibrationC &cal);

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
    std::shared_ptr<SerialComsC> m_coms;
    std::vector<ServoC> m_devices;

    std::thread m_threadMonitor;

    bool m_started = false;
    bool m_terminate = false;
  };


}

#endif
