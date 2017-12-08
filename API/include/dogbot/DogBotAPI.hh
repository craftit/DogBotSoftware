#ifndef DOGBOG_DOGBOTAPI_HEADER
#define DOGBOG_DOGBOTAPI_HEADER 1

#include "dogbot/Servo.hh"
#include <jsoncpp/json/json.h>

#include "dogbot/Coms.hh"

namespace DogBotN {

  //! Convert a fault code to a string
  const char *FaultCodeToString(FaultCodeT faultCode);

  //! Convert the calibration state to a string
  const char *HomedStateToString(MotionHomedStateT calibrationState);

  //! Convert the control mode to a string
  const char *ControlStateToString(ControlStateT controlState);

  //! Convert the control dynamic to a string
  const char *ControlDynamicToString(PWMControlDynamicT dynamic);

  //! Convert coms error to a string
  const char *ComsErrorTypeToString(ComsErrorTypeT errorCode);

  //! Convert coms packet type to a string
  const char *ComsPacketTypeToString(ComsPacketTypeT packetType);

  //! Dogbot device control

  //! This does low level management of the robot, configuration of the drivers and status monitoring.

  class DogBotAPIC
  {
  public:
    enum ServoUpdateTypeT {
      SUT_Add,
      SUT_Remove,
      SUT_Updated
    };

    enum DeviceMasterModeT {
      DMM_DeviceManager,
      DMM_ClientOnly
    };

    //! Constructor
    DogBotAPIC(const std::string &configFile = "");

    //! Construct with a string
    DogBotAPIC(const std::string &conName,std::shared_ptr<spdlog::logger> &log,DeviceMasterModeT devMaster);

    //! Construct with coms object
    DogBotAPIC(const std::shared_ptr<ComsC> &coms,std::shared_ptr<spdlog::logger> &log,bool manageComs = false,DeviceMasterModeT devMasterMode = DMM_ClientOnly);

    //! Destructor to wait for shutdown
    ~DogBotAPIC();

    //! Connect to a named device
    bool Connect(const std::string &name);

    //! Connect to coms object.
    bool Connect(const std::shared_ptr<ComsC> &coms);

    //! Access connection
    std::shared_ptr<ComsC> Connection();

    //! Set the logger to use
    void SetLogger(std::shared_ptr<spdlog::logger> &log);

    //! Start API with given config
    bool Init(const std::string &configFile);

    //! Start API
    bool Init();

    //! Load a configuration file
    bool LoadConfig(const std::string &configFile);

    //! Save configuration of roboto
    bool SaveConfig(const std::string &configFile);

    //! Read calibration from a device.
    bool ReadCalibration(int deviceId,MotorCalibrationC &cal);

    //! Write calibration to a device.
    bool WriteCalibration(int deviceId,const MotorCalibrationC &cal);

    //! Set the handler for servo reports for a device.
    int SetServoUpdateHandler(int deviceId,const std::function<void (const PacketServoReportC &report)> &handler);

    //! Add callback for state changes.
    // Called with device id and update type.
    int AddServoStatusHandler(const std::function<void (int,ServoUpdateTypeT)> &callback);

    //! Remove handler.
    void RemoveServoStatusHandler(int id);

    //! Tell all servos to hold the current position
    void DemandHoldPosition();

    //! Reset all controllers.
    void ResetAll();

    //! Refresh all controllers.
    void RefreshAll();

    //! Get servo entry by id
    std::shared_ptr<ServoC> GetServoById(int id);

    //! Get servo entry by name
    std::shared_ptr<ServoC> GetServoByName(const std::string &name);

    //! Get list of configured servos
    std::vector<std::shared_ptr<ServoC> > ListServos();

    //! Shutdown controller.
    bool Shutdown();

  protected:
    //! Issue an update notification
    void ServoStatusUpdate(int id,ServoUpdateTypeT op);

    //! Handle an incoming announce message.
    void HandlePacketAnnounce(const PacketDeviceIdC &pkt);

    //! Give unassigned devices an id.
    void ProcessUnassignedDevices();

    //! Access device id, create entry if needed
    std::shared_ptr<ServoC> DeviceEntry(int deviceId);

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

    std::mutex m_mutexStatusCallbacks;
    std::vector<std::function<void (int id,ServoUpdateTypeT)> > m_statusCallbacks;

    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");

    std::string m_deviceName;
    bool m_manageComs = false;
    Json::Value m_configRoot;
    std::shared_ptr<ComsC> m_coms;
    std::mutex m_mutexDevices;

    std::chrono::time_point<std::chrono::steady_clock> m_timeLastUnassignedUpdate;
    std::vector<std::shared_ptr<ServoC> > m_unassignedDevices; // Unassigned devices.
    std::vector<std::shared_ptr<ServoC> > m_devices; // Indexed by device id.

    std::thread m_threadMonitor;

    DeviceMasterModeT m_deviceMasterMode = DMM_ClientOnly;
    bool m_started = false;
    bool m_terminate = false;
  };


}

#endif
