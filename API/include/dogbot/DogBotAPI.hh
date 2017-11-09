#ifndef DOGBOG_DOGBOTAPI_HEADER
#define DOGBOG_DOGBOTAPI_HEADER 1

#include "dogbot/Servo.hh"
#include <json/json.h>
#include "dogbot/SerialComs.hh"

namespace DogBotN {

  //! Convert a fault code to a string
  const char *FaultCodeToString(FaultCodeT faultCode);

  //! Convert the calibration state to a string
  const char *CalibrationStateToString(MotionCalibrationT calibrationState);

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

    //! Set the handler for servo reports for a device.
    int SetServoUpdateHandler(int deviceId,const std::function<void (const PacketServoReportC &report)> &handler);

    //! Get list of configured servos
    std::vector<std::shared_ptr<ServoC> > ListServos();

    //! Shutdown controller.
    bool Shutdown();

  protected:
    //! Create a new entry for device.
    //! This assumes 'm_mutexDevices' is held.
    void CreateDeviceEntry(int deviceId);

    enum DriverStateT {
      DS_Init,
      DS_NoConnection,
      DS_Connected,
      DS_Calibrated,
      DS_Error
    } m_driverState = DS_Init;

    //! Monitor thread
    void RunMonitor();

    std::vector<int> m_comsHandlerIds;

    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");

    std::string m_deviceName;
    Json::Value m_configRoot;
    std::shared_ptr<SerialComsC> m_coms;
    std::mutex m_mutexDevices;
    std::vector<std::shared_ptr<ServoC> > m_devices; // Indexed by device id.

    std::thread m_threadMonitor;

    bool m_started = false;
    bool m_terminate = false;
  };


}

#endif
