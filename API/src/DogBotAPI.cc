
#include "dogbot/DogBotAPI.hh"
#include "dogbot/protocol.h"
#include "dogbot/ComsSerial.hh"
#include "dogbot/ComsZMQClient.hh"
#include "dogbot/ComsUSB.hh"
#include "dogbot/Joint4BarLinkage.hh"
#include "dogbot/JointRelative.hh"
#include <fstream>
#include <chrono>
#include <mutex>
#include <cassert>
#include <memory>
#include <exception>
#include <stdexcept>

namespace DogBotN {

  //! Convert a fault code to a string
  const char *FaultCodeToString(FaultCodeT faultCode)
  {
    switch(faultCode) {
    case FC_Ok: return "Ok";
    case FC_Unknown: return "Unknown";
    case FC_CalibrationFailed: return "Calibration Failed";
    case FC_DriverFault: return "Driver Fault";
    case FC_Internal: return "Internal error";
    case FC_Internal5VRailOutOfRange: return "5V Rail Out Of Range";
    case FC_InternalStoreFailed: return "Store failed";
    case FC_InternalTiming: return "Timing error";
    case FC_DriverOverTemperature: return "Over Temperature";
    case FC_OverVoltage:    return "Over Voltage";
    case FC_UnderVoltage:   return "Under Voltage";
    case FC_NoSensor:       return "No sensor";
    case FC_NoMotor:        return "No motor";
    case FC_PositionLost:   return "Position Lost";
    case FC_MotorResistanceOutOfRange: return "Resistance out of range.";
    case FC_MotorInducetanceOutOfRange: return "Inductance out of range.";
    case FC_InternalUSB: return "Internal USB error";
    case FC_InternalCAN: return "Internal CAN error";
    case FC_FanOverCurrent:return "Fan over current";
    case FC_MotorOverTemperature: return "Motor Over Temperature";
    case FC_SensorOverCurrent:return "Sensor over current";
    }

    printf("Invalid fault code %d \n", (int) faultCode);
    return "Invalid";
  }

  //! Convert the calibration state to a string
  const char *HomedStateToString(MotionHomedStateT calibrationState)
  {
    switch(calibrationState) {
      case MHS_Lost: return "Lost";
      case MHS_Measuring: return "Measuring";
      case MHS_Homed: return "Homed";
    }
    printf("Unexpected homed state %d",(int)calibrationState);
    return "Invalid";
  }

  //! Convert the control mode to a string
  const char *ControlStateToString(ControlStateT controlState)
  {
    switch(controlState) {
    case CS_EmergencyStop: return "Emergency Stop";
    case CS_FactoryCalibrate: return "Factory Calibrate";
    case CS_Standby: return "Standby";
    case CS_LowPower: return "Low Power";
    case CS_Ready: return "Ready";
    case CS_Home: return "Position Calibration";
    case CS_SelfTest: return "Self Test";
    case CS_Teach: return "Teach";
    case CS_Fault: return "Fault";
    case CS_StartUp: return "Startup";
    case CS_Diagnostic: return "Diagnostic";
    case CS_BootLoader: return "Boot Loader";
    }
    printf("Unexpected state %d",(int)controlState);
    return "Invalid";
  }

  //! Convert the control dynamic to a string
  const char *ControlDynamicToString(PWMControlDynamicT dynamic)
  {
    switch(dynamic) {
      case CM_Off: return "Off";
      case CM_Brake : return "Brake";
      case CM_Torque: return "Torque";
      case CM_Velocity: return "Velocity";
      case CM_Position: return "Position";
      case CM_Fault: return "Fault";
      case CM_Final: return "Final";
    }
    printf("Unexpected dynamic value %d",(int)dynamic);
    return "Invalid";
  }

  //! Convert coms error to a string
  const char *ComsErrorTypeToString(ComsErrorTypeT errorCode)
  {
    switch(errorCode) {
      case CET_UnknownPacketType: return "Unknown packet type";
      case CET_UnexpectedPacketSize: return "Unexpected packet size";
      case CET_ParameterOutOfRange: return "Parameter out of range ";
      case CET_CANTransmitFailed: return "CAN transmit failed";
      case CET_InternalError: return "Internal error";
      case CET_MotorNotRunning: return "Motor not running";
      case CET_NotImplemented: return "Not implemented";
      case CET_BootLoaderUnexpectedState: return "BootLoader in unexpected state";
      case CET_BootLoaderLostSequence: return "BootLoader lost sequence";
      case CET_BootLoaderErase: return "BootLoader erase";
      case CET_BootLoaderProtected: return "BootLoader protected";
      case CET_BootLoaderBusy: return "BootLoader busy";
      case CET_BootLoaderWriteFailed: return "BootLoader write failed";
      case CET_BootLoaderUnalignedAddress: return "BootLoader unaligned address";
    }
    printf("Unexpected error code %d",(int)errorCode);
    return "Invalid";
  }

  //! Convert coms packet type to a string
  const char *ComsPacketTypeToString(ComsPacketTypeT packetType)
  {
    switch(packetType) {
      case CPT_NoOp: return "No-op";
      case CPT_EmergencyStop: return "Emergency Stop";
      case CPT_SyncTime: return "SyncTime";
      case CPT_Error: return "Error";
      case CPT_SetParam: return "SetParam";
      case CPT_Servo: return "Servo";
      case CPT_ServoReport: return "ServoReport";
      case CPT_ReportParam: return "ReportParam";
      case CPT_ReadParam: return "ReadParam";
      case CPT_Pong: return "Pong";
      case CPT_Ping: return "Ping";
      case CPT_AnnounceId: return "AnnounceId";
      case CPT_QueryDevices: return "QueryDevices";
      case CPT_SetDeviceId: return "SetDeviceId";
      case CPT_SaveSetup: return "SaveSetup";
      case CPT_LoadSetup: return "LoadSetup";
      case CPT_CalZero: return "CalZero";
      case CPT_Sync: return "Sync";
      case CPT_PWMState: return "PWMState";
      case CPT_BridgeMode: return "BridgeMode";
      case CPT_FlashCmdReset: return "FlashCmdReset";
      case CPT_FlashCmdResult: return "FlashCmdResult";
      case CPT_FlashChecksumResult: return "FlashChecksumResult";
      case CPT_FlashEraseSector: return "FlashEraseSector";
      case CPT_FlashChecksum: return "FlashChecksum";
      case CPT_FlashData: return "FlashData";
      case CPT_FlashWrite: return "FlashWrite";
      case CPT_FlashRead: return "FlashRead";
    }
    printf("Unexpected packet type %d",(int)packetType);
    return "Invalid";
  }


  // ---------------------------------------------------------

  //! Constructor
  DogBotAPIC::DogBotAPIC()
  {
    m_manageComs = true;
  }

  //! Construct with coms object
  DogBotAPIC::DogBotAPIC(
      const std::shared_ptr<ComsC> &coms,
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
    if(name == "local") {
      m_coms = std::make_shared<ComsZMQClientC>("tcp://127.0.0.1");
      if(m_deviceManagerMode == DMM_Auto)
        m_deviceManagerMode = DMM_ClientOnly;
    } else if(name == "usb") {
      m_coms = std::make_shared<ComsUSBC>();
      if(m_deviceManagerMode == DMM_Auto)
        m_deviceManagerMode = DMM_DeviceManager;
    } else {
      m_coms = std::make_shared<ComsSerialC>();
      if(m_deviceManagerMode == DMM_Auto)
        m_deviceManagerMode = DMM_DeviceManager;
    }
    m_coms->SetLogger(m_log);
    if(!m_coms->Open(name.c_str()))
      return false;
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
    ComsRegisteredCallbackSetC callbacks(m_coms);
    callbacks.SetHandler(CPT_ReportParam,[this,&gotData,&toGo,&cal,&done](uint8_t *data,int size) mutable
      {
        struct PacketParam8ByteC *psp = (struct PacketParam8ByteC *) data;
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
    ComsRegisteredCallbackSetC callbacks(m_coms);
    callbacks.SetHandler(CPT_ReportParam,[this,&gotData,&toGo,&cal,&done](uint8_t *data,int size) mutable
      {
        struct PacketParam8ByteC *psp = (struct PacketParam8ByteC *) data;
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
    m_log->info("Starting API. ");
    if(m_started) {
      m_log->error("Init already called. ");
      return false;
    }

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



  //! Load a configuration file
  bool DogBotAPIC::LoadConfig(const std::string &configFile)
  {
    if(configFile.empty())
      return false;
    try {
      std::ifstream confStrm(configFile,std::ifstream::binary);

      if(!confStrm) {
        m_log->error("Failed to open configuration file '{}' ",configFile);
        return false;
      }

      Json::Value rootConfig;
      confStrm >> rootConfig;
      Json::Value kinematicsList = rootConfig["kinematics"];
      if(!kinematicsList.isNull()) {
        for(int i = 0;i < kinematicsList.size();i++) {
          Json::Value kinConf = kinematicsList[i];
          std::string name = kinConf.get("name","default").asString();
          std::shared_ptr<LegKinematicsC> kin;

          std::lock_guard<std::mutex> lock(m_mutexKinematics);
          for(auto &a : m_legKinematics) {
            if(a->Name() == name) {
              kin = a;
              break;
            }
          }
          if(!kin) {
            kin = std::make_shared<LegKinematicsC>(kinConf);
            m_legKinematics.push_back(kin);
          } else {
            kin->ConfigureFromJSON(kinConf);
          }
        }
      }

      Json::Value deviceList = rootConfig["devices"];
      if(!deviceList.isNull()) {
        ServoUpdateTypeT op = SUT_Updated;
        for(int i = 0;i < deviceList.size();i++) {
          Json::Value deviceConf = deviceList[i];
          std::string deviceType = deviceConf.get("type","").asString();
          if(!deviceType.empty() && deviceType != "servo" ) {
            std::shared_ptr<JointC> joint = MakeJoint(deviceType);
            joint->ConfigureFromJSON(*this,deviceConf);
            m_joints.push_back(joint);
            continue;
          }
          std::shared_ptr<ServoC> device;
          int deviceId = 0;
          uint32_t uid1 = deviceConf.get("uid1",0u).asInt();
          uint32_t uid2 = deviceConf.get("uid2",0u).asInt();
          int reqId = deviceConf.get("deviceId",0u).asInt();
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
                }
              } else {
                while(m_devices.size() < reqId)
                  m_devices.push_back(std::shared_ptr<ServoC>());
                deviceId = reqId;
                device = std::make_shared<ServoC>(m_coms,deviceId);
                m_devices.push_back(device);
                op = SUT_Add;
              }
              if(deviceId > 0) {
                device = std::make_shared<ServoC>(m_coms,deviceId);
                m_devices[deviceId] = device;
              }
            }
            // Still no luck... just append it.
            if(deviceId == 0) {
              deviceId = (int) m_devices.size();
              device = std::make_shared<ServoC>(m_coms,deviceId);
              m_devices.push_back(device);
              op = SUT_Add;
            }
          }
          assert(device);
          if(device) {
            device->ConfigureFromJSON(*this,deviceConf);
            ServoStatusUpdate(device.get(),op);
            if(device->IsExported())
              m_joints.push_back(device);
          }
        }
      }
    } catch(std::exception &ex) {
      m_log->error("Failed load configuration file {}", ex.what());
      return false;
    }

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

    std::lock_guard<std::mutex> lock(m_mutexDevices);
    Json::Value deviceList;
    int index =0;
    for(auto &a : m_devices) {
      if(!a)
        continue;
      deviceList[index++] = a->ConfigAsJSON();
    }
    for(auto &a : m_joints) {
      if(!a)
        continue;
      if(a->JointType() == "servo")
        continue;
      deviceList[index++] = a->ConfigAsJSON();
    }
    rootConfig["devices"] = deviceList;

    confStrm << rootConfig;

    return true;
  }


  //! Issue an update notification
  void DogBotAPIC::ServoStatusUpdate(JointC *id,ServoUpdateTypeT op)
  {
    for(auto &a : m_jointStatusCallbacks.Calls()) {
      if(a) a(id,op);
    }
  }

  //! Access device id, create entry if needed
  std::shared_ptr<ServoC> DogBotAPIC::DeviceEntry(int deviceId)
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);

    // The device clearly exists, so make sure there is an entry.
    if(deviceId >= m_devices.size() || deviceId < 0)
      return std::shared_ptr<ServoC>();
    return m_devices[deviceId];
  }

  void DogBotAPIC::HandlePacketAnnounce(const PacketDeviceIdC &pkt)
  {
    std::shared_ptr<ServoC> device;
    int deviceId = pkt.m_deviceId;
    m_log->info("Handling device announcement {} {}   Id:{} ",pkt.m_uid[0],pkt.m_uid[1],(int) pkt.m_deviceId);
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
          device = std::make_shared<ServoC>(m_coms,deviceId,pkt);
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
             device->UId1(),device->UId2(),device->Name()
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
        device = std::make_shared<ServoC>(m_coms,0,pkt);
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
      ServoStatusUpdate(device.get(),updateType);
    }
  }


  //! Monitor thread
  void DogBotAPIC::RunMonitor()
  {
    m_log->info("Running monitor. Manager mode: {} ",((m_deviceManagerMode == DMM_DeviceManager) ? "master" : "client"));
    //logger->info("Starting bark");

    if(!m_coms) {
      m_log->error("No coms object, aborting monitor. ");
      return ;
    }

    ComsRegisteredCallbackSetC callbacks(m_coms);

    callbacks.SetHandler(CPT_Pong, [this](uint8_t *data,int size) mutable
    {
      struct PacketPingPongC *pkt = (struct PacketPingPongC *) data;
      m_log->info("Got pong from {} ",(int) pkt->m_deviceId);
    });

    callbacks.SetHandler(CPT_ServoReport,
             [this](uint8_t *data,int size) mutable
        {
          if(size != sizeof(struct PacketServoReportC)) {
            m_log->error("Unexpected 'ServoReport' packet length {} ",size);
            return;
          }
          const PacketServoReportC *pkt = (const PacketServoReportC *) data;
          std::shared_ptr<ServoC> device = DeviceEntry(pkt->m_deviceId);
          if(!device)
            return ;
          if(device->HandlePacketServoReport(*pkt)) {
            ServoStatusUpdate(device.get(),SUT_Updated);
          }
        }
    );

    callbacks.SetHandler(
          CPT_AnnounceId,
           [this](uint8_t *data,int size) mutable
            {
              if(size != sizeof(struct PacketDeviceIdC)) {
                m_log->error("Unexpected 'AnnounceId' packet length {} ",size);
                return;
              }
              const PacketDeviceIdC *pkt = (const PacketDeviceIdC *) data;
              HandlePacketAnnounce(*pkt);
            }
           );

    callbacks.SetHandler(CPT_ReportParam,
                       [this](uint8_t *data,int size) mutable
                        {
                          if(size < sizeof(struct PacketParamHeaderC)) {
                            m_log->error("'ReportParam' packet to short {} ",size);
                            return;
                          }
                          const PacketParam8ByteC *pkt = (const PacketParam8ByteC *) data;
                          // We can only deal with devices after they've been allocated an id.
                          std::shared_ptr<ServoC> device = DeviceEntry(pkt->m_header.m_deviceId);
                          if(!device)
                            return ;
                          if(device->HandlePacketReportParam(*pkt)) {
                            ServoStatusUpdate(device.get(),SUT_Updated);
                          }

                        }
                       );

    callbacks.SetHandler(CPT_Error,
                       [this](uint8_t *data,int size) mutable
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

                          std::shared_ptr<ServoC> device = DeviceEntry(pkt->m_deviceId);
                          if(!device)
                            return ;
#if 0
                          if(device->HandlePacketReportParam(*pkt)) {
                            ServoStatusUpdate(pkt->m_header.m_deviceId,SUT_Updated);
                          }
#endif

                        }
                       );

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
          m_coms->SendQueryDevices();
          m_coms->SendPing(0); // Find out what we're connected to.
        }
        // no break
        case DS_Connected: {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          if(!m_coms->IsReady()) {
            m_driverState = DS_NoConnection;
            break;
          }
          auto now = std::chrono::steady_clock::now();
          bool unassignedDevicesFound = false;
          std::vector<std::shared_ptr<ServoC> > devicesList;
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
              ServoStatusUpdate(a.get(),SUT_Updated);
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
    std::vector<JointC *> updated;
    {
      std::lock_guard<std::mutex> lock(m_mutexDevices);
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
        m_log->info("Adding new device '{}' ",deviceId);
        device->SetId(deviceId);
        updated.push_back(device.get());
        m_coms->SendSetDeviceId(deviceId,device->UId1(),device->UId2());
      }
    }
    for(auto a : updated)
      ServoStatusUpdate(a,op);
  }


  //! Tell all servos to hold the current position
  void DogBotAPIC::DemandHoldPosition()
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
          servo = m_devices[i];
      }
      if(!servo || servo->Id() == 0)
        continue;
      m_coms->SendSetParam(servo->Id(),CPI_PWMMode,CM_Position);
      servo->DemandPosition(servo->Position(),servo->DefaultPositionTorque());
    }
    // Make sure
  }

  void DogBotAPIC::ResetAll()
  {
    m_coms->SendSetParam(0,CPI_ControlState,CS_StartUp);

  }

  void DogBotAPIC::RefreshAll()
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
          servo = m_devices[i];
      }
      if(!servo || servo->Id() == 0)
        continue;
      servo->QueryRefresh();
    }

  }

  //! Get servo entry by id
  std::shared_ptr<ServoC> DogBotAPIC::GetServoById(int id)
  {
    if(id < 0)
      return std::shared_ptr<ServoC>();
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    if(id >= m_devices.size())
      return std::shared_ptr<ServoC>();
    return m_devices[id];
  }

  //! Get servo entry by name
  std::shared_ptr<JointC> DogBotAPIC::GetJointByName(const std::string &name)
  {
    std::lock_guard<std::mutex> lock(m_mutexKinematics);
    for(auto &a : m_devices) {
      if(a && a->Name() == name)
        return a;
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
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    for(auto &a : m_legKinematics) {
      if(a && a->Name() == name)
        return a;
    }
    return std::shared_ptr<LegKinematicsC>();
  }


  //! Get list of configured servos
  std::vector<std::shared_ptr<ServoC> > DogBotAPIC::ListServos()
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    return m_devices;
  }


  //! Get list of joints
  std::vector<std::shared_ptr<JointC> > DogBotAPIC::ListJoints()
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    return m_joints;
  }

}
