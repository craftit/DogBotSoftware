
#include "dogbot/DogBotAPI.hh"
#include "dogbot/protocol.h"
#include <fstream>
#include <chrono>
#include <mutex>
#include <cassert>
#include <memory>

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
    case FC_OverTemprature: return "Over Temprature";
    case FC_OverVoltage:    return "Over Voltage";
    case FC_UnderVoltage:   return "Under Voltage";
    case FC_NoSensor:       return "No sensor";
    case FC_NoMotor:        return "No motor";
    case FC_PositionLost:   return "Position Lost";
    case FC_MotorResistanceOutOfRange: return "Resistance out of range.";
    case FC_MotorInducetanceOutOfRange: return "Inductance out of range.";
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
    default: {
      printf("Unexpected state %d",(int)controlState);
      return "Invalid";
    }
    }
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
      default:
      case CM_Final: return "Final";
    }
  }

  // ---------------------------------------------------------

  //! Constructor
  DogBotAPIC::DogBotAPIC(const std::string &configFile)
  {
    m_manageComs = true;
    if(!configFile.empty())
      LoadConfig(configFile);
  }

  //! Construct with coms object
  DogBotAPIC::DogBotAPIC(
      const std::shared_ptr<ComsC> &coms,
      std::shared_ptr<spdlog::logger> &log,
      bool manageComs
      )
   : m_coms(coms)
  {
    m_manageComs = manageComs;
    m_log = log;
    m_coms->SetLogger(log);
  }

  DogBotAPIC::~DogBotAPIC()
  {
    m_terminate = true;
    m_threadMonitor.join();
  }

  //! Set the logger to use
  void DogBotAPIC::SetLogger(std::shared_ptr<spdlog::logger> &log)
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

  //! Connect to coms object.
  bool DogBotAPIC::Connect(const std::shared_ptr<ComsC> &coms)
  {
    assert(!m_coms);
    m_coms = coms;
    return true;
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
    int packetProc = m_coms->SetHandler(CPT_ReportParam,[this,&gotData,&toGo,&cal,&done](uint8_t *data,int size) mutable
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
    int packetProc = m_coms->SetHandler(CPT_ReportParam,[this,&gotData,&toGo,&cal,&done](uint8_t *data,int size) mutable
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

  //! Is this the master server, responsible for managing device ids ?
  //! Default is false.
  void DogBotAPIC::SetMaster(bool val)
  {
    m_isMaster = val;
  }

  //! Start API with given config
  bool DogBotAPIC::Init(const std::string &configFile)
  {
    if(!LoadConfig(configFile))
      return false;
    return Init();
  }

  //! Load configuration for the robot.

  bool DogBotAPIC::Init()
  {
    m_log->info("Starting API. ");
    if(m_started) {
      m_log->error("Init already called. ");
      return false;
    }

    //! Initalise which serial device to use.
    m_deviceName = m_configRoot.get("device", "/dev/ttyACM0" ).asString();

    m_started = true;

    m_threadMonitor = std::move(std::thread { [this]{ RunMonitor(); } });

    return true;
  }

  //! Load a configuration file
  bool DogBotAPIC::LoadConfig(const std::string &configFile)
  {
    if(configFile.empty())
      return false;

    std::ifstream confStrm(configFile,std::ifstream::binary);

    if(!confStrm) {
      m_log->error("Failed to open configuration file '{}' ",configFile);
      return false;
    }

    Json::Value rootConfig;
    confStrm >> rootConfig;
    Json::Value deviceList = rootConfig["devices"];
    if(!deviceList.isNull()) {
      ServoUpdateTypeT op = SUT_Updated;
      for(int i = 0;i < deviceList.size();i++) {
        Json::Value deviceConf = deviceList[i];
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
          ServoStatusUpdate(deviceId,op);
        }
      }
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
    rootConfig["devices"] = deviceList;

    confStrm << rootConfig;

    return true;
  }


  //! Issue an update notification
  void DogBotAPIC::ServoStatusUpdate(int id,ServoUpdateTypeT op)
  {
    std::lock_guard<std::mutex> lock(m_mutexStatusCallbacks);
    for(auto &a : m_statusCallbacks) {
      if(a) a(id,op);
    }
  }

  //! Add callback for state changes.
  int DogBotAPIC::AddServoStatusHandler(const std::function<void (int id,ServoUpdateTypeT)> &callback)
  {
    std::lock_guard<std::mutex> lock(m_mutexStatusCallbacks);
    // Free slot anywhere?
    for(int i = 0;i < (int) m_statusCallbacks.size();i++) {
      if(!m_statusCallbacks[i]) {
        m_statusCallbacks[i] = callback;
        return i;
      }
    }
    // Just create a new one
    int ret = (int) m_statusCallbacks.size();
    m_statusCallbacks.push_back(callback);
    return ret;
  }

  //! Remove handler.
  void DogBotAPIC::RemoveServoStatusHandler(int id)
  {
    assert(id >= 0);
    assert(id < m_statusCallbacks.size());
    std::lock_guard<std::mutex> lock(m_mutexStatusCallbacks);
    m_statusCallbacks[id] = std::function<void (int id,ServoUpdateTypeT)>();
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
    m_log->info("Handling device announcement {} {}   Id:{}",pkt.m_uid[0],pkt.m_uid[1],(int) pkt.m_deviceId);
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
        device = std::make_shared<ServoC>(m_coms,deviceId,pkt);
        m_devices[deviceId] = device;
        updateType = SUT_Add;
      } else {
        // Check existing device id matches
        if(!device->HasUID(pkt.m_uid[0],pkt.m_uid[1])) {
          // Conflicting id's detected.
          deviceId = 0; // Treat device as if it is unassigned.
          if(m_isMaster)
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
      if(!device && m_isMaster) {
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
    assert(device);
    device->HandlePacketAnnounce(pkt,m_isMaster);
    ServoStatusUpdate(deviceId,updateType);
  }


  //! Monitor thread
  void DogBotAPIC::RunMonitor()
  {
    m_log->info("Running monitor. ");
    //logger->info("Starting bark");

    m_coms->SetHandler(CPT_Pong, [this](uint8_t *data,int size) mutable
    {

    });

    m_coms->SetHandler(CPT_ServoReport,
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
            ServoStatusUpdate(pkt->m_deviceId,SUT_Updated);
          }
        }
    );

    m_coms->SetHandler(
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

    m_coms->SetHandler(CPT_ReportParam,
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
                            ServoStatusUpdate(pkt->m_header.m_deviceId,SUT_Updated);
                          }

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
          if(m_manageComs) {
            if(!m_coms->Open(m_deviceName.c_str())) {
              m_log->warn("Failed to open coms channel. ");
              std::this_thread::sleep_for(std::chrono::seconds(2));
              break;
            }
          }
          if(!m_coms->IsReady()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            break;
          }
          m_driverState = DS_Connected;
          // Send out a device query.
          m_coms->SendQueryDevices();
        }
        // no break
        case DS_Connected: {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          if(!m_coms->IsReady()) {
            m_driverState = DS_NoConnection;
          }
          auto now = std::chrono::steady_clock::now();
          bool unassignedDevicesFound = false;
          std::vector<std::shared_ptr<ServoC> > devicesList;
          {
            std::lock_guard<std::mutex> lock(m_mutexDevices);
            devicesList = m_devices;
            if(m_unassignedDevices.size() > 0) {
              std::chrono::duration<double> elapsed_seconds = now-m_timeLastUnassignedUpdate;
              unassignedDevicesFound = (elapsed_seconds.count() > 0.2);
            }
          }
          for(auto &a : devicesList) {
            if(!a)
              continue;
            if(a->UpdateTick(now)) {
              ServoStatusUpdate(a->Id(),SUT_Updated);
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
    std::vector<int> updated;
    {
      std::lock_guard<std::mutex> lock(m_mutexDevices);
      while(m_unassignedDevices.size() > 0) {
        auto device = m_unassignedDevices.back();
        m_unassignedDevices.pop_back();
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
        updated.push_back(deviceId);
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
  std::shared_ptr<ServoC> DogBotAPIC::GetServoByName(const std::string &name)
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    for(auto &a : m_devices) {
      if(a->Name() == name)
        return a;
    }
    return std::shared_ptr<ServoC>();
  }


  //! Set the handler for servo reports for a device.
  int DogBotAPIC::SetServoUpdateHandler(int deviceId,const std::function<void (const PacketServoReportC &report)> &handler)
  {
    assert(0); // Not implemented
    return 0;
  }


  //! Get list of configured servos
  std::vector<std::shared_ptr<ServoC> > DogBotAPIC::ListServos()
  {
    std::lock_guard<std::mutex> lock(m_mutexDevices);
    std::vector<std::shared_ptr<ServoC> > ret = m_devices;
    return ret;
  }

}
