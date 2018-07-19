#ifndef DOGBOG_DOGBOTAPI_HEADER
#define DOGBOG_DOGBOTAPI_HEADER 1

#include "dogbot/Servo.hh"
#include <jsoncpp/json/json.h>
#include "dogbot/Coms.hh"
#include "dogbot/LegKinematics.hh"
#include "dogbot/CallbackArray.hh"
#include "dogbot/DogBotKinematics.hh"

namespace DogBotN {

  //! Exception thrown if we find an error in a configuration file.

  class ExceptionBadConfigC
   : public std::exception
  {
  public:
    ExceptionBadConfigC(const char *msg)
      : m_msg(msg)
    {}

    virtual const char *what() const noexcept override
    { return m_msg; }
  protected:
    const char *m_msg;
  };

  class LegControllerC;

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

    enum DeviceManagerModeT {
      DMM_DeviceManager,
      DMM_ClientOnly,
      DMM_Auto
    };

    //! Default constructor
    //! This leaves it up to the caller to load any configuration with 'LoadConfig'
    //! Connect to the required device with 'Connect' if not specified in the configuration file.
    //! then call 'Init' to start processing. 'Init' must be called before the rest of the interface is used.
    DogBotAPIC();

    //! Construct with a string
    //! \param connectionName default is 'local' for local server.  'usb' for direct usb connection,'none' to nothing,'tcp://[ipaddress]' to connect to remote machine
    //! \param log Where to output log messages
    //! \param devMaster If this instance of the class should manage device ids, management is enabled if connecting directly via usb, and not otherwise
    DogBotAPIC(
        const std::string &connectionName,
        const std::string &configurationFile = "",
        const std::shared_ptr<spdlog::logger> &log = spdlog::stdout_logger_mt("console"),
        DeviceManagerModeT devManager = DMM_Auto
        );

    //! Construct with coms object
    //! \param coms Communication object to use.
    //! \param log Where to send log messages
    //! \param manageComs - Set to true if you want the API to manage reconnection, false otherwise
    //! \param devMaster If this instance of the class should manage device ids, management is enabled if connecting directly via usb, and not otherwise
    DogBotAPIC(
        const std::shared_ptr<ComsC> &coms,
        const std::string &configurationFile,
        const std::shared_ptr<spdlog::logger> &log = spdlog::stdout_logger_mt("console"),
        bool manageComs = false,
        DeviceManagerModeT devMasterMode = DMM_Auto
        );

    //! Destructor to wait for shutdown
    ~DogBotAPIC();

    //! Get the time now
    static double TimeNow();

    //! Find the default configuration file.
    //! Returns an empty string if one wasn't found
    static std::string DefaultConfigFile(const std::string &robotName = "");

    //! Connect to a named device
    //! Normally 'usb' or 'local', 'none' for nothing.
    bool Connect(const std::string &name);

    //! Connect using an existing communication object.
    bool Connect(const std::shared_ptr<ComsC> &coms);

    //! Access current connection
    std::shared_ptr<ComsC> Connection();

    //! Set the logger to use
    void SetLogger(const std::shared_ptr<spdlog::logger> &log);

    //! Load a configuration file
    bool LoadConfig(const std::string &configFile);

    //! Save configuration of robot
    bool SaveConfig(const std::string &configFile);

    //! Start API
    //! return true if started ok. It will return false if API has already been initialised.
    bool Init();

    //! Home all the joints in the robot
    //! Returns true if succeeded
    bool HomeAll();

    //! Tell all servos to hold the current position
    void DemandHoldPosition();

    //! Reset all controllers.
    void PowerOnAll();

    //! Request all controllers go into low power mode
    void StandbyAll();

    //! Make the robot go limp by disabling all motors
    void MotorsOffAll();

    //! Switch the break on for all motors
    void BrakeAll();

    //! Refresh information on all controllers.
    void RefreshAll();

    //! Initiate an emergency stop
    void EmergencyStop();

    //! Set velocity limit for all connected servos.
    void SetVelocityLimit(float velocityLimit);

    //! Set all supply voltage calibration values to 1
    bool SetSupplyVoltageScaleToOne();

    //! Update supply voltage calibration
    bool CalibrateSupplyVoltage(float supplyVoltage);

    //! Restore configuration from file values.
    bool RestoreConfig();

    //! Get device entry by name
    std::shared_ptr<DeviceC> GetDeviceByName(const std::string &name);

    //! Get servo entry by id
    std::shared_ptr<ServoC> GetServoById(int id);

    //! Get servo entry by name
    std::shared_ptr<JointC> GetJointByName(const std::string &name);

    //! Get joint entry by name
    std::shared_ptr<JointC> GetJointById(int id);

    //! Get kinematics for leg by name
    std::shared_ptr<LegKinematicsC> LegKinematicsByName(const std::string &name);

    //! Access dogbot kinematics
    const DogBotKinematicsC &DogBotKinematics() const
    { return m_dogBotKinematics; }

    //! Access dogbot kinematics
    DogBotKinematicsC &DogBotKinematics()
    { return m_dogBotKinematics; }

    //! Get list of configured servos
    //! These are actual devices on the robot. These are 'actuators' in ROS speak.
    std::vector<std::shared_ptr<ServoC> > ListServos();

    //! Get a list of virtual joints giving a simplified picture of the robot.
    //! The 4 bar linkage used for the knees is converted to look like a set of independent joints.
    std::vector<std::shared_ptr<JointC> > ListJoints();

    //! Shutdown controller.
    bool Shutdown();

    //! Logger currently in use
    spdlog::logger &Log()
    { return *m_log; }

    //! Access shared pointer to log
    std::shared_ptr<spdlog::logger> &LogPtr()
    { return m_log; }

    //! Add callback for state changes. When a servo is added, removed or updated.
    // Called with device id and update type.
    CallbackHandleC AddServoStatusHandler(const std::function<void (JointC *,ServoUpdateTypeT)> &callback)
    { return m_jointStatusCallbacks.Add(callback); }

    //! Add callback for state changes. When a servo is added, removed or updated.
    // Called with device id and update type.
    CallbackHandleC AddDeviceStatusHandler(const std::function<void (DeviceC *,ServoUpdateTypeT)> &callback)
    { return m_deviceStatusCallbacks.Add(callback); }

    //! Access an ordered list of the four leg names
    static const std::vector<std::string> &LegNames()
    { return DogBotKinematicsC::LegNames(); }

    //! Access names of leg joints.
    static const std::vector<std::string> &LegJointNames()
    { return DogBotKinematicsC::LegJointNames(); }

  protected:
    //! Home from squatting position (Not implemented yet)
    bool HomeSquat();

    // Call a method for all connected servos
    bool ForAllServos(const std::function<void (ServoC *)> &func);

    // Call a method for all connected devices
    bool ForAllDevices(const std::function<void (DeviceC *)> &func);

    //! Make a new device
    std::shared_ptr<DeviceC> MakeDevice(int deviceId,const std::string &deviceTypeName);

    //! Make a new device
    std::shared_ptr<DeviceC> MakeDevice(int deviceId,DeviceTypeT deviceType);

    //! Make device for given id
    std::shared_ptr<DeviceC> MakeDevice(int deviceId,const PacketDeviceIdC &pkt);

    //! Read calibration from a device.
    bool ReadCalibration(int deviceId,MotorCalibrationC &cal);

    //! Write calibration to a device.
    bool WriteCalibration(int deviceId,const MotorCalibrationC &cal);

    //! Issue an update notification
    void DeviceStatusUpdate(DeviceC *,ServoUpdateTypeT op);

    //! Handle an incoming announce message.
    void HandlePacketAnnounce(const PacketDeviceIdC &pkt);

    //! Give unassigned devices an id.
    void ProcessUnassignedDevices();

    //! Make an appropriate type of device.
    std::shared_ptr<JointC> MakeJoint(const std::string &jointType) const;

    //! Access device id, create entry if needed
    std::shared_ptr<DeviceC> DeviceEntry(int deviceId);

    //! Access servo id, create entry if needed
    std::shared_ptr<ServoC> ServoEntry(int deviceId);

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

    CallbackArrayC<std::function<void (DeviceC *,ServoUpdateTypeT)> > m_deviceStatusCallbacks;
    CallbackArrayC<std::function<void (JointC *,ServoUpdateTypeT)> > m_jointStatusCallbacks;

    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");

    std::string m_deviceName;
    bool m_manageComs = false;
    Json::Value m_configRoot;
    std::shared_ptr<ComsC> m_coms;
    std::mutex m_mutexDevices;

    std::chrono::time_point<std::chrono::steady_clock> m_timeLastUnassignedUpdate;
    std::vector<std::shared_ptr<DeviceC> > m_unassignedDevices; // Unassigned devices.
    std::vector<std::shared_ptr<DeviceC> > m_devices; // Indexed by device id.
    std::vector<std::shared_ptr<JointC> > m_joints; // List of available joints.

    std::thread m_threadMonitor;

    DeviceManagerModeT m_deviceManagerMode = DMM_ClientOnly;
    bool m_started = false;
    bool m_terminate = false;
    DogBotKinematicsC m_dogBotKinematics;
    float m_lastSpeedLimit = 0;
    std::vector<bool> m_emergencyStopFlags = std::vector<bool>(256,false);
  };


}

#endif
