
#include "dogbot/DogBotAPI.hh"
#include "dogbot/protocol.h"
#include "dogbot/ComsSerial.hh"
#include "dogbot/ComsZMQClient.hh"
#include "dogbot/ComsUSB.hh"
#include "dogbot/ComsProxy.hh"
#include "dogbot/Joint4BarLinkage.hh"
#include "dogbot/JointRelative.hh"
#include "dogbot/DeviceIMU.hh"
#include "dogbot/DevicePlatformManager.hh"
#include "dogbot/Strings.hh"

#include <fstream>
#include <chrono>
#include <mutex>
#include <cassert>
#include <memory>
#include <exception>
#include <stdexcept>
#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#define DODEBUG 0
#if DODEBUG
#define ONDEBUG(x) x
#else
#define ONDEBUG(x)
#endif

namespace DogBotN {

  //! Constructor
  DogBotAPIC::DogBotAPIC()
  {
    m_manageComs = true;
  }

  //! Construct with coms object
  DogBotAPIC::DogBotAPIC(
      const std::shared_ptr<ComsC> &coms,
      const std::string &configurationFile,
      const std::shared_ptr<spdlog::logger> &log,
      bool manageComs,
      DeviceManagerModeT devMasterMode
      )
   : m_coms(coms),
     m_deviceManagerMode(devMasterMode)
  {
    m_manageComs = manageComs;
    m_log = log;
    m_coms->SetLogger(log);
    if(!configurationFile.empty()) {
      LoadConfig(configurationFile);
    }
    Init();
  }

  //! Construct with a string
  DogBotAPIC::DogBotAPIC(
      const std::string &connectionName,
      const std::string &configurationFile,
      const std::shared_ptr<spdlog::logger> &log,
      DeviceManagerModeT devMasterMode
      )
    : m_deviceManagerMode(devMasterMode)
  {
    m_log = log;
    m_manageComs = true;
    if(!connectionName.empty())
      Connect(connectionName);
    if(!configurationFile.empty()) {
      LoadConfig(configurationFile);
    }
    Init();
  }


  DogBotAPIC::~DogBotAPIC()
  {
    m_terminate = true;
    if(m_threadMonitor.joinable())
      m_threadMonitor.join();
  }

  //! Get the time now
  double DogBotAPIC::TimeNow()
  {
    return TimePointT::clock::now().time_since_epoch().count();
  }

  //! Set the logger to use
  void DogBotAPIC::SetLogger(const std::shared_ptr<spdlog::logger> &log)
  {
    m_log = log;
    m_coms->SetLogger(log);
  }


  //! Shutdown controller.
  bool DogBotAPIC::Shutdown()
  {
    m_terminate = true;
    return false;
  }

  //! Connect to a named device
  bool DogBotAPIC::Connect(const std::string &name)
  {
    m_deviceName = name;

    auto colonAt = name.find(':');
    std::string prefix;
    if(colonAt != std::string::npos) {
      prefix = name.substr(0,colonAt);
    }
    bool doOpen = true;
    //std::cerr << "Got prefix '" << prefix << "' " << std::endl;
    if(prefix == "tcp" || prefix == "udp") {
      m_coms = std::make_shared<ComsZMQClientC>(name);
      if(m_deviceManagerMode == DMM_Auto)
        m_deviceManagerMode = DMM_ClientOnly;
    } else  if(name == "local") {
      m_coms = std::make_shared<ComsZMQClientC>("tcp://127.0.0.1");
      if(m_deviceManagerMode == DMM_Auto)
        m_deviceManagerMode = DMM_ClientOnly;
    } else if(name == "none") {
      m_coms = std::make_shared<ComsProxyC>();
      if(m_deviceManagerMode == DMM_Auto)
        m_deviceManagerMode = DMM_ClientOnly;
      doOpen = false;
    } else if(name == "usb") {
      //m_coms = std::make_shared<ComsUSBC>();
      if(m_deviceManagerMode == DMM_Auto)
        m_deviceManagerMode = DMM_DeviceManager;
      m_log->error("Direct USB connection not supported.");
      return false;
    } else {
      m_coms = std::make_shared<ComsSerialC>();
      if(m_deviceManagerMode == DMM_Auto)
        m_deviceManagerMode = DMM_DeviceManager;
    }
    m_coms->SetLogger(m_log);
    if(doOpen) {
      if(!m_coms->Open(name.c_str()))
        return false;
    }
    {
      std::lock_guard<std::mutex> lock(m_mutexDevices);
      for(auto &a : m_devices)
        if(a) a->UpdateComs(m_coms);
    }
    return true;
  }

  //! Connect to coms object.
  bool DogBotAPIC::Connect(const std::shared_ptr<ComsC> &coms)
  {
    assert(!m_coms);
    m_coms = coms;
    return true;
  }

  //! Access connection
  std::shared_ptr<ComsC> DogBotAPIC::Connection()
  {
    return m_coms;
  }

  //! Read calibration from a device.
  bool DogBotAPIC::ReadCalibration(int deviceId,MotorCalibrationC &cal)
  {
    if(!m_coms)
      return false;

    int toGo = 12;
    std::vector<bool> gotData(toGo,false);
    std::timed_mutex done;
    done.lock();
    CallbackSetC callbacks;
    callbacks += m_coms->SetHandler(CPT_ReportParam,[this,&gotData,&toGo,&cal,&done](const uint8_t *data,int size) mutable
      {
        const struct PacketParam8ByteC *psp = (const struct PacketParam8ByteC *) data;
        int index = psp->m_header.m_index;
        if(index < CPI_ANGLE_CAL || index > CPI_ANGLE_CAL_11)
          return ;
        index -= (int) CPI_ANGLE_CAL;
        //m_log->info("Cal data. {} : {} {} {}",index,psp->m_data.uint16[0],psp->m_data.uint16[1],psp->m_data.uint16[2]);

        cal.SetCal(index,psp->m_data.uint16[0],psp->m_data.uint16[1],psp->m_data.uint16[2]);
        if(!gotData[index]) {
          toGo--;
          gotData[index] = true;
          if(toGo == 0)
            done.unlock();
        }
      });
    bool ret = false;
    for(int i = 0;i < 4;i++) {
      // Send query for each parameter
      for(int i = 0;i < 12;i++) {
        if(!gotData[i])
          m_coms->SendQueryParam(deviceId,static_cast<enum ComsParameterIndexT>(CPI_ANGLE_CAL + i));
      }
      // Did we get them all ?
      if(done.try_lock_for(std::chrono::milliseconds(200))) {
        ret = true;
        //m_log->info("Got calibration data.");
        break;
      }
    }

    return ret;
  }

  //! Write calibration to a device.
  bool DogBotAPIC::WriteCalibration(int deviceId,const MotorCalibrationC &cal)
  {
    if(!m_coms)
      return false;

    int toGo = 12;
    std::vector<bool> gotData(toGo,false);
    std::timed_mutex done;
    done.lock();
    CallbackSetC callbacks;
    callbacks += m_coms->SetHandler(CPT_ReportParam,[this,&gotData,&toGo,&cal,&done](const uint8_t *data,int size) mutable
      {
        const struct PacketParam8ByteC *psp = (const struct PacketParam8ByteC *) data;
        int index = psp->m_header.m_index;
        if(index < CPI_ANGLE_CAL || index > CPI_ANGLE_CAL_11)
          return ;
        index -= (int) CPI_ANGLE_CAL;
        //m_log->info("Cal data. {} : {} {} {}",index,psp->m_data.uint16[0],psp->m_data.uint16[1],psp->m_data.uint16[2]);
        if(!gotData[index]) {

          uint16_t h1,h2,h3;
          cal.GetCal(index,h1,h2,h3);
          bool updateOk = (h1 == psp->m_data.uint16[0]) && (h2 == psp->m_data.uint16[1]) && (h3 == psp->m_data.uint16[2]);
          if(updateOk) {
            toGo--;
            gotData[index] = true;
            if(toGo == 0)
              done.unlock();
          }
        }
      });

    bool ret = false;
    for(int i = 0;i < 4;i++) {
      // Send query for each parameter
      for(int i = 0;i < 12;i++) {
        if(!gotData[i]) {
          //m_coms->SendQueryParam(deviceId,static_cast<enum ComsParameterIndexT>(CPI_ANGLE_CAL + i));
          BufferTypeT buff;
          cal.GetCal(i,buff.uint16[0],buff.uint16[1],buff.uint16[2]);
          m_coms->SendSetParam(deviceId,static_cast<enum ComsParameterIndexT>(CPI_ANGLE_CAL + i),buff,6);
          // Query value to check it was set correctly.
          m_coms->SendQueryParam(deviceId,static_cast<enum ComsParameterIndexT>(CPI_ANGLE_CAL + i));
        }
      }
      // Did we set them all ok ?
      if(done.try_lock_for(std::chrono::milliseconds(200))) {
        ret = true;
        //m_log->info("Got calibration data.");
        break;
      }
    }

    return true;
  }


  //! Load configuration for the robot.

  bool DogBotAPIC::Init()
  {
    ONDEBUG(m_log->info("Starting API. "));
    if(m_started) {
      ONDEBUG(m_log->error("Init already called. "));
      return false;
    }

    for(int i = 0;i < m_emergencyStopFlags.size();i++)
      m_emergencyStopFlags[i] = false;

    //! Initialise which serial device to use.
    if(m_deviceName.empty())
      m_deviceName = m_configRoot.get("device","usb").asString();
    if(m_deviceManagerMode == DMM_Auto && !m_deviceName.empty()) {
      if(m_deviceName == "local") {
        m_deviceManagerMode = DMM_ClientOnly;
      } else {
        m_deviceManagerMode = DMM_DeviceManager;
      }
    }
    m_started = true;

    m_threadMonitor = std::move(std::thread { [this]{ RunMonitor(); } });

    return true;
  }


  //! Home from squatting position
  bool DogBotAPIC::HomeSquat()
  {
//            "back_left_knee" : 238.32269287109375,
//            "back_left_pitch" : -58.273262023925781,
//            "back_left_roll" : 0,
//            "back_right_knee" : 121.75420379638672,
//            "back_right_pitch" : 58.361156463623047,
//            "back_right_roll" : 0,
//            "front_left_knee" : 238.30073547363281,
//            "front_left_pitch" : -59.393905639648438,
//            "front_left_roll" : 0,
//            "front_right_knee" : 121.66630554199219,
//            "front_right_pitch" : 59.591667175292969,
//            "front_right_roll" : 0

#if 0
    const std::vector<std::string> &legNames = LegNames();
    // First check all the roll joints are in the right place.
    {

      std::shared_ptr<ServoC> jnts[4];
      for(int i = 0;i < 4;i++) {
        std::string jointName = legNames[i] + "_roll";
        jnts[i] = std::dynamic_pointer_cast<ServoC>(GetJointByName(jointName));
        if(!jnts[i]->IsAtHomeIndex()) {
          m_log->warn("Roll joint {} not in correct position.",jointName);
          return false;
        }
      }

      // Each of the roll drives should at the zero position
      for(int i = 0;i < 4;i++) {
        TimePointT tick;
        double position = 0;
        double velocity = 0;
        double torque = 0;
        enum PositionReferenceT posRef = PR_Absolute;
        if(!jnts[i]->GetRawState(tick,position,velocity,torque,posRef)) {
          m_log->warn("Failed to query {} servo position.",jnts[i]->Name());
          return false;
        }
        // If joint is already homed, skip it.
        if(posRef != PR_Relative) {
          continue;
        }
        m_coms->SendSetParam(jnts[i]->Id(),CPI_homeIndexPosition,position);
      }

      //m_homeIndexState
    }
#endif


    return true;
  }

  //! Home whole robot;
  bool DogBotAPIC::HomeAll()
  {
    const std::vector<std::string> &legNames = LegNames();
    bool allOk = true;
    {
      std::vector<std::thread> threads;

      // Home roll
      for(int i = 0;i < legNames.size();i++) {
        std::string jointName = legNames[i] + "_roll";
        std::shared_ptr<ServoC> jnt = std::dynamic_pointer_cast<ServoC>(GetJointByName(jointName));
        if(!jnt) {
          m_log->error("Failed to find joint {} ",jointName);
          return false;
        }
        threads.push_back(std::thread([jnt,&allOk](){
          if(!jnt->HomeJoint(false)) {
            allOk = false;
            return false;
          }
          jnt->DemandPosition(0,2.0,PR_Absolute);
          return true;
        }));
      }
      for(auto &thr: threads) {
        if(thr.joinable())
          thr.join();
      }
      if(!allOk)
        return false;
    }

    {
      std::vector<std::thread> threads;
      // Home knees.
      HomeDirectionHintT hints[4] = { HDH_Clockwise,HDH_Anticlockwise,HDH_Anticlockwise,HDH_Clockwise };

      for(int i = 0;i < legNames.size();i++) {
        std::string jointName = legNames[i] + "_knee";
        std::shared_ptr<ServoC> jnt = std::dynamic_pointer_cast<ServoC>(GetJointByName(jointName));
        if(!jnt) {
          m_log->error("Failed to find joint {} ",jointName);
          return false;
        }
        HomeDirectionHintT hint = hints[i];
        threads.push_back(std::thread([jnt,&allOk,hint](){
          if(!jnt->HomeJoint(false,hint)) {
            allOk = false;
            return false;
          }
          jnt->DemandPosition(0,2.0,PR_Absolute);
          return true;
        }));
      }
      for(auto &thr: threads) {
        if(thr.joinable())
          thr.join();
      }
      if(!allOk)
        return false;
    }

    {
      std::vector<std::thread> threads;
      HomeDirectionHintT hints[4] = { HDH_Anticlockwise,HDH_Clockwise,HDH_Clockwise,HDH_Anticlockwise };

      // Home pitch
      for(int i = 0;i < legNames.size();i++) {
        std::string jointName = legNames[i] + "_pitch";
        std::shared_ptr<ServoC> jnt = std::dynamic_pointer_cast<ServoC>(GetJointByName(jointName));
        if(!jnt) {
          m_log->error("Failed to find joint {} ",jointName);
          return false;
        }
        HomeDirectionHintT hint = hints[i];
        threads.push_back(std::thread([jnt,&allOk,hint](){
          if(!jnt->HomeJoint(false,hint)) {
            allOk = false;
            return false;
          }
          jnt->DemandPosition(0,2.0,PR_Absolute);
          return true;
        }));
      }
      for(auto &thr: threads) {
        if(thr.joinable())
          thr.join();
      }
      if(!allOk)
        return false;
    }

    return true;
  }

  //! Make an appropriate type of device.
  std::shared_ptr<JointC> DogBotAPIC::MakeJoint(const std::string &deviceType) const
  {
    if(deviceType == "relative") {
      return std::make_shared<JointRelativeC>();
    } else if(deviceType == "4bar") {
      return std::make_shared<Joint4BarLinkageC>();
    }
    m_log->error("Unknown joint type '{}'",deviceType);
    assert(0);
    throw std::runtime_error("Unknown joint type.");
  }


  static bool FileExists(const std::string &filename)
  {
    struct stat sb;
    return stat(filename.c_str(),&sb) ==  0;
  }


  //! Find the default configuration file
  std::string DogBotAPIC::DefaultConfigFile(const std::string &robotName)
  {
    std::string devFilename = "local";
    std::string homeDir = getenv("HOME");

    std::string theRobotName = robotName;
    if(theRobotName.empty())
      theRobotName = "robot";
    std::string defaultConfig = homeDir + "/.config/dogbot/" + theRobotName + ".json";
    if(!FileExists(defaultConfig))  {
      std::cerr << "Default configuration file '" << defaultConfig << "' doesn't exist. " << std::endl;
      return "";
    }

    return defaultConfig;
  }

  //! Make a new device
  std::shared_ptr<DeviceC> DogBotAPIC::MakeDevice(int deviceId,const std::string &deviceTypeName)
  {
    if(deviceTypeName == "servo" || deviceTypeName == "Servo")
      return MakeDevice(deviceId,DT_MotorDriver);
    if(deviceTypeName == "imu" || deviceTypeName == "IMU")
      return MakeDevice(deviceId,DT_IMU);
    if(deviceTypeName == "PlatformManager")
      return MakeDevice(deviceId,DT_PlatformManager);
    m_log->error("Asked to create unknown device type '{}' ",deviceTypeName);
    return MakeDevice(deviceId,DT_Unknown);
  }

  //! Make a new device
  std::shared_ptr<DeviceC> DogBotAPIC::MakeDevice(int deviceId,DeviceTypeT deviceType)
  {
    ONDEBUG(m_log->info("Creating device {} of type '{}' ({}) ",deviceId,ComsDeviceTypeToString(deviceType),(int) deviceType));
    switch(deviceType)
    {
      case DT_Unknown: return std::make_shared<DeviceC>(m_coms,deviceId);
      case DT_MotorDriver: return std::make_shared<ServoC>(m_coms,deviceId);
      case DT_PlatformManager:  return std::make_shared<DevicePlatformManagerC>(m_coms,deviceId);
      case DT_BootLoader: return std::make_shared<DeviceC>(m_coms,deviceId);
      case DT_IMU: return std::make_shared<DeviceIMUC>(m_coms,deviceId);
        break;
    }
    m_log->warn("Asked to create unknown device type {} ",(int) deviceType);
    return std::make_shared<DeviceC>(m_coms,deviceId);
  }


  //! Make device for given id
  std::shared_ptr<DeviceC> DogBotAPIC::MakeDevice(int deviceId,const PacketDeviceIdC &pkt)
  {
    DeviceTypeT deviceType = (DeviceTypeT) ((int) pkt.m_idBytes[3] >> 4);
    ONDEBUG(m_log->info("Creating device {} of type '{}' ({}) ID: 0x{:08x} 0x{:08x}  ",deviceId,ComsDeviceTypeToString(deviceType),deviceType,pkt.m_uid[0],pkt.m_uid[1]));
    switch(deviceType)
    {
      case DT_BootLoader:
      case DT_MotorDriver:
      case DT_Unknown: return std::make_shared<ServoC>(m_coms,deviceId,pkt);
      case DT_IMU: return std::make_shared<DeviceIMUC>(m_coms,deviceId,pkt);
      case DT_PlatformManager: return std::make_shared<DevicePlatformManagerC>(m_coms,deviceId,pkt);
    }
    m_log->warn("Asked to create unknown device type {} ",(int) deviceType);
    return std::make_shared<DeviceC>(m_coms,deviceId,pkt);
  }

  //! Load a configuration file
  bool DogBotAPIC::LoadConfig(const std::string &configFile)
  {
    if(configFile.empty())
      return false;
#if 0
    try {
#endif
      std::ifstream confStrm(configFile,std::ifstream::binary);

      if(!confStrm) {
        m_log->error("Failed to open configuration file '{}' ",configFile);
        return false;
      }

      Json::Value rootConfig;
      confStrm >> rootConfig;

      m_dogBotKinematics.ConfigureFromJSON(rootConfig);

      Json::Value deviceList = rootConfig["devices"];
      if(!deviceList.isNull()) {
        ServoUpdateTypeT op = SUT_Updated;
        for(int i = 0;i < deviceList.size();i++) {
          Json::Value deviceConf = deviceList[i];
          std::string jointType = deviceConf.get("type","").asString();
          if(!jointType.empty() && (
              jointType == "relative" ||
              jointType == "4bar" ||
              jointType == "joint"))
          {
            std::shared_ptr<JointC> joint = MakeJoint(jointType);
            joint->ConfigureFromJSON(*this,deviceConf);
            //m_log->info("Loaded joint '{}' of type '{}' ",joint->Name(),jointType);
            m_joints.push_back(joint);
            continue;
          }
          std::shared_ptr<DeviceC> device;
          int deviceId = 0;
          uint32_t uid1 = deviceConf.get("uid1",0u).asUInt();
          uint32_t uid2 = deviceConf.get("uid2",0u).asUInt();

          int reqId = deviceConf.get("deviceId",0u).asInt();
          std::string deviceType = deviceConf.get("device_type","servo").asString();
          // Ignore silly ids
          if(reqId > 255 || reqId < 0)
            reqId = 0;
          {
            std::lock_guard<std::mutex> lock(m_mutexDevices);
            // Is the device present ?
            for(int i = 0;i < m_devices.size();i++) {
              if(!m_devices[i])
                continue;
              if(m_devices[i]->HasUID(uid1,uid2)) {
                deviceId = i;
                device = m_devices[i];
                break;
              }
            }
            // Go with the requested configuration id if we can.
            if(deviceId == 0 && reqId != 0) {
              if(m_devices.size() > reqId) {
                if(!m_devices[reqId]) {
                  deviceId = reqId;
                  if(deviceId > 0) {
                    device = MakeDevice(deviceId,deviceType);
                    m_devices[deviceId] = device;
                  }
                }
              } else {
                while(m_devices.size() < reqId)
                  m_devices.push_back(std::shared_ptr<ServoC>());
                deviceId = reqId;
                device = MakeDevice(deviceId,deviceType);
                m_devices.push_back(device);
                op = SUT_Add;
              }
            }
            // Still no luck... just append it.
            if(deviceId == 0) {
              deviceId = (int) m_devices.size();
              device = MakeDevice(deviceId,deviceType);
              m_devices.push_back(device);
              op = SUT_Add;
            }
          }
          assert(device);
          if(device) {
            device->ConfigureFromJSON(*this,deviceConf);
            ONDEBUG(m_log->info("Loaded device '{}' of type '{}' ",device->DeviceName(),deviceType));
            DeviceStatusUpdate(device.get(),op);
            std::shared_ptr<JointC> jnt = std::dynamic_pointer_cast<JointC>(device);
            if(jnt) m_joints.push_back(jnt);
          }
        }
      }
#if 0
    } catch(std::exception &ex) {
      m_log->error("Failed to load configuration file. : {}", ex.what());
      return false;
    }
#endif

    return true;
  }

  //! Save configuration of robot

  bool DogBotAPIC::SaveConfig(const std::string &configFile)
  {
    std::ofstream confStrm(configFile,std::ifstream::binary);

    if(!confStrm) {
      m_log->error("Failed to open configuration file '{}' ",configFile);
      return false;
    }
    Json::Value rootConfig;

    {
      std::lock_guard<std::mutex> lock(m_mutexDevices);
      Json::Value deviceList;
      int index =0;
      for(auto &a : m_devices) {
        if(!a)
          continue;
        a->ConfigAsJSON(deviceList[index++]);
      }
      for(auto &a : m_joints) {
        if(!a)
          continue;
        if(a->JointType() == "servo")
          continue;
        a->ConfigAsJSON(deviceList[index++]);
      }
      rootConfig["devices"] = deviceList;

      //! Get the servo configuration as JSON
      m_dogBotKinematics.ConfigAsJSON(rootConfig);
    }

    confStrm << rootConfig;

    return true;
  }


  //! Issue an update notification
  void DogBotAPIC::DeviceStatusUpdate(DeviceC *id,ServoUpdateTypeT op)
  {
    for(auto &a : m_deviceStatusCallbacks.Calls()) {
      if(a) a(id,op);
    }
    JointC *jnt = dynamic_cast<JointC *>(id);
    if(jnt != 0) {
      for(auto &a : m_jointStatusCallbacks.Calls()) {
        if(a) a(jnt,op);
      }
    }
  }

  //! Access servo by deviceId
  std::shared_ptr<ServoC> DogBotAPIC::ServoEntry(int deviceId)
  {
    std::shared_ptr<ServoC> ret;
    std::lock_guard<std::mutex> lock(m_mutexDevices);

    // The device clearly exists, so make sure there is an entry.
    if(deviceId >= m_devices.size() || deviceId < 0)
      return std::shared_ptr<ServoC>();
    ret = std::dynamic_pointer_cast<ServoC>(m_devices[deviceId]);
    if(!ret && m_devices[deviceId])
      m_log->warn("Device {} not a servo. ",deviceId);
    return ret;
  }

  //! Access by device id
  std::shared_ptr<DeviceC> DogBotAPIC::DeviceEntry(int deviceId)
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);

    // The device clearly exists, so make sure there is an entry.
    if(deviceId >= m_devices.size() || deviceId < 0)
      return std::shared_ptr<DeviceC>();
    return m_devices[deviceId];
  }

  void DogBotAPIC::HandlePacketAnnounce(const PacketDeviceIdC &pkt)
  {
    std::shared_ptr<DeviceC> device;
    int deviceId = pkt.m_deviceId;
    DeviceTypeT deviceType = (DeviceTypeT) ((int) pkt.m_idBytes[3] >> 4);
    if(deviceType == DT_Unknown)
      deviceType = DT_MotorDriver;
    if(pkt.m_deviceId == 0)
      m_log->info("Handling device announcement 0x{:08x} 0x{:08x} Type:{} Id:{} ",pkt.m_uid[0],pkt.m_uid[1],ComsDeviceTypeToString(deviceType),(int) pkt.m_deviceId);
    ServoUpdateTypeT updateType = SUT_Updated;
    // Check device id is
    if(deviceId != 0) {
      // Check device ids match
      device = DeviceEntry(deviceId);
      // If no entry found.
      if(!device) {
        assert(deviceId <= 255 && deviceId >= 0);

        // Add entry to table with existing id.
        std::lock_guard<std::mutex> lock(m_mutexDevices);
        while(m_devices.size() <= deviceId)
          m_devices.push_back(nullptr);
        // Look through things previously flagged as unassigned.
        for(int i = 0;i < m_unassignedDevices.size();i++) {
          if(m_unassignedDevices[i]->HasUID(pkt.m_uid[0],pkt.m_uid[1])) {
            // Found it!
            device = m_unassignedDevices[i];
            m_unassignedDevices.erase(m_unassignedDevices.begin() + i);
            break;
          }
        }
        if(!device)
          device = MakeDevice(deviceId,pkt);
        m_devices[deviceId] = device;
        updateType = SUT_Add;
      } else {
        // Check existing device id matches
        if(!device->HasUID(pkt.m_uid[0],pkt.m_uid[1])) {
          // Conflicting id's detected.
          deviceId = 0; // Treat device as if it is unassigned.
          if(m_deviceManagerMode == DMM_DeviceManager)
            m_coms->SendSetDeviceId(0,pkt.m_uid[0],pkt.m_uid[1]); // Stop it using the conflicting address.
          m_log->error(
             "Conflicting ids detected for device {}, with uid {}-{} and existing device {}-{} '{}' ",
             (int) pkt.m_deviceId,pkt.m_uid[0],pkt.m_uid[1],
             device->UId1(),device->UId2(),device->DeviceName()
             );
        }
      }
    }
    if(deviceId == 0) {
      // Look through devices for a matching id.
      std::lock_guard<std::mutex> lock(m_mutexDevices);
      // 0 is a reserved id, so start from 1
      for(int i = 1;i < m_devices.size();i++) {
        if(m_devices[i] && m_devices[i]->HasUID(pkt.m_uid[0],pkt.m_uid[1])) {
          deviceId = i;
          device = m_devices[i];
        }
      }
      // No matching id found ?
      if(!device && m_deviceManagerMode == DMM_DeviceManager) {
        device = MakeDevice(0,pkt);
        m_timeLastUnassignedUpdate = std::chrono::steady_clock::now();
        for(auto &a : m_unassignedDevices) {
          if(a->HasUID(pkt.m_uid[0],pkt.m_uid[1])) {
            return ; // Already in list.
          }
        }
        m_unassignedDevices.push_back(device);
        return ;
      }
    }
    assert(device || m_deviceManagerMode != DMM_DeviceManager);
    if(device) {
      device->HandlePacketAnnounce(pkt,m_deviceManagerMode == DMM_DeviceManager);
      DeviceStatusUpdate(device.get(),updateType);
    }
  }


  //! Monitor thread
  void DogBotAPIC::RunMonitor()
  {
    ONDEBUG(m_log->info("Running monitor. Manager mode: {} ",((m_deviceManagerMode == DMM_DeviceManager) ? "master" : "client")));

    if(!m_coms) {
      m_log->error("No coms object, aborting monitor. ");
      return ;
    }

    CallbackSetC callbacks;

    callbacks += m_coms->SetHandler(
        CPT_Pong, [this](const uint8_t *data,int size) mutable
    {
      struct PacketPingPongC *pkt = (struct PacketPingPongC *) data;
      std::shared_ptr<DeviceC> device = DeviceEntry(pkt->m_deviceId);
      if(device) device->HandlePacketPong(pkt);
      ONDEBUG(m_log->info("Got pong from {} ",(int) pkt->m_deviceId));
    });

    callbacks += m_coms->SetHandler(
        CPT_ServoReport,
             [this](const uint8_t *data,int size) mutable
        {
          if(size != sizeof(struct PacketServoReportC)) {
            m_log->error("Unexpected 'ServoReport' packet length {} ",size);
            return;
          }
          const PacketServoReportC *pkt = (const PacketServoReportC *) data;
          std::shared_ptr<ServoC> device = ServoEntry(pkt->m_deviceId);
          if(!device)
            return ;
          if(device->HandlePacketServoReport(*pkt)) {
            DeviceStatusUpdate(device.get(),SUT_Updated);
          }
        }
    );

    callbacks += m_coms->SetHandler(
          CPT_AnnounceId,
           [this](const uint8_t *data,int size) mutable
            {
              if(size != sizeof(struct PacketDeviceIdC)) {
                m_log->error("Unexpected 'AnnounceId' packet length {} ",size);
                return;
              }
              const PacketDeviceIdC *pkt = (const PacketDeviceIdC *) data;
              HandlePacketAnnounce(*pkt);
            }
           );

    callbacks += m_coms->SetHandler(
        CPT_ReportParam,
                       [this](const uint8_t *data,int size) mutable
                        {
                          if(size < sizeof(struct PacketParamHeaderC)) {
                            m_log->error("'ReportParam' packet to short {} ",size);
                            return;
                          }
                          const PacketParam8ByteC *pkt = (const PacketParam8ByteC *) data;
                          // Reset
                          if(((enum ComsParameterIndexT) pkt->m_header.m_index) == CPI_ControlState &&
                              ((enum ControlStateT) pkt->m_data.int8[0]) == CS_Standby) {
                            m_emergencyStopFlags[pkt->m_header.m_deviceId] = false;
                          }
                          // We can only deal with devices after they've been allocated an id.
                          std::shared_ptr<DeviceC> device = DeviceEntry(pkt->m_header.m_deviceId);
                          if(!device)
                            return ;
                          if(device->HandlePacketReportParam(*pkt,size)) {
                            DeviceStatusUpdate(device.get(),SUT_Updated);
                          }

                        }
                       );

    callbacks += m_coms->SetHandler(CPT_Error,
                       [this](const uint8_t *data,int size) mutable
                        {
                          if(size < sizeof(struct PacketErrorC)) {
                            m_log->error("'Error' packet to short {} ",size);
                            return;
                          }
                          const PacketErrorC *pkt = (const PacketErrorC *) data;

                          m_log->error("Received packet error code from device {} : {} ({}) Packet:{} ({}) Arg:{} ",
                                       (int) pkt->m_deviceId,
                                       DogBotN::ComsErrorTypeToString((enum ComsErrorTypeT)pkt->m_errorCode),
                                       (int) pkt->m_errorCode,
                                       DogBotN::ComsPacketTypeToString((enum ComsPacketTypeT) pkt->m_causeType),(int) pkt->m_causeType,
                                       (int) pkt->m_errorData);
                        }
                       );

    callbacks += m_coms->SetHandler(CPT_IMU,
                       [this](const uint8_t *data,int size) mutable
                        {
                          if(size != sizeof(PacketIMUC)) {
                            m_log->error("'IMU' packet unexpected length {} ",size);
                            return ;
                          }
                          struct PacketIMUC *pkt = (struct PacketIMUC *) data;
                          std::shared_ptr<DeviceC> device = DeviceEntry(pkt->m_deviceId);
                          if(!device)
                            return ;
                          DeviceC *devPtr = device.get();
                          DeviceIMUC *anIMU = dynamic_cast<DeviceIMUC *>(devPtr);
                          if(anIMU == 0) {
                            m_log->error("IMU Packet from non-IMU device. {} ({}) ",(int) pkt->m_deviceId,typeid(*devPtr).name());
                            return;
                          }
                          if(anIMU->HandlePacketIMU(pkt)) {
                            DeviceStatusUpdate(device.get(),SUT_Updated);
                          }
                        });

    callbacks += m_coms->SetHandler(CPT_Range,
                       [this](const uint8_t *data,int size) mutable
                        {
                          if(size != sizeof(PacketRangeC)) {
                            m_log->error("'IMU' packet unexpected length {} ",size);
                            return ;
                          }
                          struct PacketRangeC *pkt = (struct PacketRangeC *) data;
                          std::shared_ptr<DeviceC> device = DeviceEntry(pkt->m_deviceId);
                          if(!device)
                            return ;
                          DeviceC *devPtr = device.get();
                          DeviceIMUC *anIMU = dynamic_cast<DeviceIMUC *>(devPtr);
                          if(anIMU == 0) {
                            m_log->error("Range Packet from non-IMU device. {} ({}) ",(int) pkt->m_deviceId,typeid(*devPtr).name());
                            return;
                          }
                          if(anIMU->HandlePacketRange(pkt)) {
                            DeviceStatusUpdate(device.get(),SUT_Updated);
                          }
                        });

    callbacks += m_coms->SetHandler(CPT_Message,
                                    [this](const uint8_t *data,int size) mutable
                                     {
                                       if(size < 2) {
                                         m_log->info("Empty message received. ");
                                         return ;
                                       }
                                       char buff[256];
                                       memcpy(buff,&data[2],size-2);
                                       buff[size-2] = 0;
                                       m_log->info("Message from {} : {} ",(int) data[1],buff);
                                     });

    callbacks += m_coms->SetHandler(CPT_EmergencyStop,
                                    [this](const uint8_t *data,int size) mutable
                                     {
                                       PacketEmergencyStopC *es =  (struct PacketEmergencyStopC *) data;
                                       if(size != sizeof(PacketEmergencyStopC)) {
                                         m_log->error("Unexpected emergency stop packet size {} ",size);
                                         return ;
                                       }
                                       if(m_emergencyStopFlags[es->m_deviceId]) // Already seen message?
                                         return ;
                                       m_emergencyStopFlags[es->m_deviceId] = true;
                                       std::string causeStr;
                                       if(es->m_cause & 0x80) {
                                         causeStr = std::string("Device ") + std::to_string((int)(es->m_cause & 0x7f));
                                       } else {
                                         causeStr = ComsStateChangeSource((enum StateChangeSourceT) es->m_cause);
                                       }
                                       m_log->error("Got emergency stop from device {}, cause type {} ({}) ",(int) es->m_deviceId,causeStr,(int) es->m_cause);
                                     });

    while(!m_terminate)
    {
      //m_log->info("State. {} ",(int) m_driverState);
      switch(m_driverState)
      {
        case DS_Init: {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          m_driverState = DS_NoConnection;
        } break;
        case DS_NoConnection: {
          if(!m_coms->IsReady()) {
            if(m_deviceName == "none") {
              std::this_thread::sleep_for(std::chrono::seconds(2));
              break;
            }
            if(m_manageComs && !m_coms->Open(m_deviceName.c_str())) {
              m_log->warn("Failed to open coms channel. ");
              std::this_thread::sleep_for(std::chrono::seconds(2));
              break;
            }
            if(!m_coms->IsReady()) {
              std::this_thread::sleep_for(std::chrono::seconds(1));
              break;
            }
          }
          m_driverState = DS_Connected;
          // Send out a device query.
          for(int i = 0;i < 3;i++)
            m_coms->SendSync();
          m_coms->SendPing(0); // Find out what we're connected to.
          m_coms->SendQueryDevices();
        }
        // no break
        case DS_Connected: {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          if(!m_coms->IsReady()) {
            m_driverState = DS_NoConnection;
            break;
          }
          auto now = std::chrono::steady_clock::now();
          //m_coms->SendPing(0); // Find out what we're connected to.
          bool unassignedDevicesFound = false;
          std::vector<std::shared_ptr<DeviceC> > devicesList;
          {
            std::lock_guard<std::mutex> lock(m_mutexDevices);
            devicesList = m_devices;
            if(m_unassignedDevices.size() > 0) {
              std::chrono::duration<double> elapsed_seconds = now-m_timeLastUnassignedUpdate;
              //std::cerr << "Time: " << elapsed_seconds.count() << std::endl;
              unassignedDevicesFound = (elapsed_seconds.count() > 0.5);
            }
          }
          for(auto &a : devicesList) {
            if(!a)
              continue;
            if(a->UpdateTick(now)) {
              DeviceStatusUpdate(a.get(),SUT_Updated);
            }
          }
          if(unassignedDevicesFound)
            ProcessUnassignedDevices();
        } break;
        case DS_Calibrated:
        case DS_Error:
          std::this_thread::sleep_for(std::chrono::seconds(1));
          break;
      }
    }

    m_log->debug("Exiting monitor. ");
  }

  void DogBotAPIC::ProcessUnassignedDevices()
  {
    ServoUpdateTypeT op = SUT_Add;
    std::vector<DeviceC *> updated;
    {
      std::unique_lock<std::mutex> lock(m_mutexDevices);
      while(m_unassignedDevices.size() > 0) {
        auto device = m_unassignedDevices.back();
        m_unassignedDevices.pop_back();
        // Has device been an assigned an id after
        // it was initially queued ?
        if(device->Id() != 0) {
          // A bit of paranoia to make sure things aren't getting confused.
          assert(m_devices.size() > device->Id());
          assert(m_devices[device->Id()] == device);
          continue;
        }
        int deviceId = 0;
        // 0 is a reserved id, so start from 1
        for(int i = 1;i < m_devices.size();i++) {
          if(!m_devices[i]) {
            deviceId = i;
            op = SUT_Updated;
            break;
          }
        }
        if(deviceId == 0) {
          deviceId = m_devices.size();
          m_devices.push_back(device);
        } else {
          m_devices[deviceId] = device;
        }
        m_log->info("Adding new device {} of type '{}' named '{}' ",deviceId,device->DeviceType(),device->DeviceName());
        device->SetId(deviceId);
        updated.push_back(device.get());
        lock.unlock();
        m_coms->SendSetDeviceId(deviceId,device->UId1(),device->UId2());
        lock.lock();
      }
    }
    for(auto a : updated)
      DeviceStatusUpdate(a,op);
  }


  // Call a method for all connected servos
  bool DogBotAPIC::ForAllServos(const std::function<void (ServoC *)> &func)
  {
    bool ret = true;
    size_t deviceCount = 0;
    {
      std::lock_guard<std::mutex> lock(m_mutexDevices);
      deviceCount = m_devices.size();
    }
    for(int i = 0;i < deviceCount;i++) {
      std::shared_ptr<ServoC> servo;
      {
        std::lock_guard<std::mutex> lock(m_mutexDevices);
        deviceCount = m_devices.size();
        if(i < deviceCount)
          servo = std::dynamic_pointer_cast<ServoC>(m_devices[i]);
      }
      if(!servo || servo->Id() == 0)
        continue;
      func(servo.get());
    }
    return ret;
  }

  // Call a method for all connected devices
  bool DogBotAPIC::ForAllDevices(const std::function<void (DeviceC *)> &func)
  {
    size_t deviceCount = 0;
    {
      std::lock_guard<std::mutex> lock(m_mutexDevices);
      deviceCount = m_devices.size();
    }
    for(int i = 0;i < deviceCount;i++) {
      std::shared_ptr<DeviceC> dev;
      {
        std::lock_guard<std::mutex> lock(m_mutexDevices);
        deviceCount = m_devices.size();
        if(i < deviceCount)
          dev = m_devices[i];
      }
      if(!dev || dev->Id() == 0)
        continue;
      func(dev.get());
    }

    return true;
  }

  //! Tell all servos to hold the current position
  void DogBotAPIC::DemandHoldPosition()
  {
    ForAllServos([this](ServoC *servo) {
      m_coms->SendSetParam(servo->Id(),CPI_PWMMode,CM_Position);
      servo->DemandPosition(servo->Position(),servo->DefaultPositionTorque(),servo->PositionReference());
    });
    // Make sure
  }

  void DogBotAPIC::PowerOnAll()
  {
    m_coms->SendSetParam(0,CPI_ControlState,CS_StartUp);
    for(int i = 0;i < m_emergencyStopFlags.size();i++)
      m_emergencyStopFlags[i] = false;
  }

  //! Request all controllers go into low power mode
  void DogBotAPIC::StandbyAll()
  {
    m_coms->SendSetParam(0,CPI_ControlState,CS_Standby);
    for(int i = 0;i < m_emergencyStopFlags.size();i++)
      m_emergencyStopFlags[i] = false;
  }

  //! Make the robot go limp by disabling all motors
  void DogBotAPIC::MotorsOffAll()
  {
    m_coms->SendSetParam(0,CPI_PWMMode,CM_Off);
  }

  //! Switch the break on for all motors
  void DogBotAPIC::BrakeAll()
  {
    m_coms->SendSetParam(0,CPI_PWMMode,CM_Brake);
  }

  void DogBotAPIC::RefreshAll()
  {
    ForAllDevices([](DeviceC *dev){ dev->QueryRefresh(); });
  }

  //! Initiate an emergency stop

  void DogBotAPIC::EmergencyStop()
  {
    // Send several in case of lost packets.
    for(int i = 0;i < 10;i++) {
      m_coms->SendEmergencyStop();
      usleep(10000);
    }
  }

  //! Set velocity limit for all connected servos.
  void DogBotAPIC::SetVelocityLimit(float velocityLimit)
  {
    m_coms->SendSetParam(0,CPI_VelocityLimit,velocityLimit);
  }


  //! Update supply voltage calibration
  bool DogBotAPIC::SetSupplyVoltageScaleToOne()
  {
    size_t deviceCount = 0;
    {
      std::lock_guard<std::mutex> lock(m_mutexDevices);
      deviceCount = m_devices.size();
    }
    for(int i = 0;i < deviceCount;i++) {
      std::shared_ptr<ServoC> servo;
      {
        std::lock_guard<std::mutex> lock(m_mutexDevices);
        deviceCount = m_devices.size();
        if(i < deviceCount)
          servo = std::dynamic_pointer_cast<ServoC>(m_devices[i]);
      }
      if(!servo || servo->Id() == 0)
        continue;
      float scale = 1.0;
      m_log->warn("Device {} ({}) setting scale to {}. ",servo->Name(),servo->Id(),scale);
      usleep(1000);
      m_coms->SendSetParam(servo->Id(),CPI_SupplyVoltageScale,scale);
    }
    return true;
  }

  //! Update supply voltage calibration
  bool DogBotAPIC::CalibrateSupplyVoltage(float supplyVoltage)
  {
    size_t deviceCount = 0;
    if(supplyVoltage < 10.0 || supplyVoltage > 40.0) {
      m_log->warn("Supply voltage calibration must be between 10 and 40 volts.");
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(m_mutexDevices);
      deviceCount = m_devices.size();
    }
    for(int i = 0;i < deviceCount;i++) {
      std::shared_ptr<ServoC> servo;
      {
        std::lock_guard<std::mutex> lock(m_mutexDevices);
        deviceCount = m_devices.size();
        if(i < deviceCount)
          servo = std::dynamic_pointer_cast<ServoC>(m_devices[i]);
      }
      if(!servo || servo->Id() == 0)
        continue;
      float supplyReading = servo->SupplyVoltage();
      if(supplyReading < 10.0) { // Must be above 10 voltage to calibrate correctly.
        m_log->warn("Device {} has a supply voltage below 10 volts, skipping. ",servo->Id());
        continue;
      }
      float scale = supplyVoltage / (supplyReading * servo->SupplyVoltageScale());
      m_log->warn("Device {} ({}) setting scale to {}. ",servo->Name(),servo->Id(),scale);
      m_coms->SendSetParam(servo->Id(),CPI_SupplyVoltageScale,scale);
      m_coms->SendStoreConfig(servo->Id());
      usleep(1000);
    }
    return true;
  }

  //! Restore configuration from file values.
  bool DogBotAPIC::RestoreConfig()
  {
    size_t deviceCount = 0;
    {
      std::lock_guard<std::mutex> lock(m_mutexDevices);
      deviceCount = m_devices.size();
    }
    bool ok = true;
    for(int i = 0;i < deviceCount;i++) {
      std::shared_ptr<ServoC> servo;
      {
        std::lock_guard<std::mutex> lock(m_mutexDevices);
        deviceCount = m_devices.size();
        if(i < deviceCount)
          servo = std::dynamic_pointer_cast<ServoC>(m_devices[i]);
      }
      if(!servo || servo->Id() == 0)
        continue;
      if(!servo->RestoreConfig())
        ok = true;
    }
    return ok;
  }

  //! Get servo entry by id
  std::shared_ptr<ServoC> DogBotAPIC::GetServoById(int id)
  {
    if(id < 0)
      return std::shared_ptr<ServoC>();
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    if(id >= m_devices.size())
      return std::shared_ptr<ServoC>();
    return std::dynamic_pointer_cast<ServoC>(m_devices[id]);
  }

  //! Get joint entry by name
  std::shared_ptr<JointC> DogBotAPIC::GetJointById(int id)
  {
    return std::shared_ptr<JointC>(GetServoById(id));
  }

  //! Get device entry by name
  std::shared_ptr<DeviceC> DogBotAPIC::GetDeviceByName(const std::string &name)
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    for(auto &a : m_devices) {
      if(a && a->DeviceName() == name)
        return a;
    }
    return std::shared_ptr<DeviceC>();
  }

  //! Get servo entry by name
  std::shared_ptr<JointC> DogBotAPIC::GetJointByName(const std::string &name)
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    for(auto &a : m_devices) {
      if(a && a->DeviceName() == name)
        return std::dynamic_pointer_cast<JointC>(a);
    }
    for(auto &a : m_joints) {
      if(a && a->Name() == name)
        return a;
    }
    return std::shared_ptr<JointC>();
  }

  //! Get kinematics for leg by name
  std::shared_ptr<LegKinematicsC> DogBotAPIC::LegKinematicsByName(const std::string &name)
  {
    return m_dogBotKinematics.LegKinematicsByName(name);
  }

  //! Get list of configured servos
  std::vector<std::shared_ptr<ServoC> > DogBotAPIC::ListServos()
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    std::vector<std::shared_ptr<ServoC> > servos;
    for(auto &a : m_devices) {
      std::shared_ptr<ServoC> ptr = std::dynamic_pointer_cast<ServoC>(a);
      if(ptr)
        servos.push_back(ptr);
    }
    return servos;
  }


  //! Get list of joints
  std::vector<std::shared_ptr<JointC> > DogBotAPIC::ListJoints()
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    return m_joints;
  }


}
