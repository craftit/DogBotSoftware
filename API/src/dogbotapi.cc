
#include "../include/dogbot/DogBotAPI.hh"
#include <fstream>
#include <chrono>
#include <mutex>
#include <assert.h>

namespace DogBotN {

  //! Convert a fault code to a string
  const char *FaultCodeToString(FaultCodeT faultCode)
  {
    switch(faultCode) {
    case FC_Ok: return "Ok";
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
    }

    printf("Invalid fault code %d \n", (int) faultCode);
    return "Invalid";
  }

  //! Convert the calibration state to a string
  const char *CalibrationStateToString(MotionCalibrationT calibrationState)
  {
    switch(calibrationState) {
      case MC_Uncalibrated: return "Uncalibrated";
      case MC_Measuring: return "Measuring";
      case MC_Calibrated: return "Calibrated";
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
    case CS_PositionCalibration: return "Position Calibration";
    case CS_SelfTest: return "Self Test";
    case CS_Teach: return "Teach";
    case CS_Fault: return "Fault";
    case CS_StartUp: return "Startup";
    default: {
      printf("Unexpected state %d",(int)controlState);
      return "Invalid";
    }
    }
  }

  // ---------------------------------------------------------

  //! Constructor
  DogBotAPIC::DogBotAPIC(const std::string &configFile)
  {
    m_manageComs = true;
    if(!configFile.empty())
      LoadConfig(configFile);
    Init();
  }

  //! Construct with coms object
  DogBotAPIC::DogBotAPIC(const std::shared_ptr<SerialComsC> &coms)
   : m_coms(coms)
  {
    m_manageComs = false;
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
  bool DogBotAPIC::Connect(const std::shared_ptr<SerialComsC> &coms)
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

    std::ifstream confStrm(configFile,std::ifstream::binary);

    if(!confStrm) {
      m_log->error("Failed to open configuration file '{}' ",configFile);
      return false;
    }

    confStrm >> m_configRoot;
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
    if(deviceId >= m_devices.size() ||
        !m_devices[deviceId]) {
      m_log->info("Adding new device '{}' ",deviceId);

      assert(deviceId <= 255 && deviceId >= 0);

      while(m_devices.size() <= deviceId)
        m_devices.push_back(nullptr);

      std::shared_ptr<ServoC> &ptr = m_devices[deviceId];
      if(!ptr) {
        ptr = std::make_shared<ServoC>(m_coms,deviceId);
      }

      ServoStatusUpdate(deviceId,SUT_Add);
    }

    return  m_devices[deviceId];

  }

  //! Monitor thread
  void DogBotAPIC::RunMonitor()
  {
    m_log->debug("Running monitor. ");

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
          if(device->HandlePacketServoReport(*pkt)) {
            ServoStatusUpdate(pkt->m_deviceId,SUT_Updated);
          }
        }
    );

    m_coms->SetHandler(CPT_AnnounceId,
                       [this](uint8_t *data,int size) mutable
                        {
                          if(size != sizeof(struct PacketDeviceIdC)) {
                            m_log->error("Unexpected 'AnnounceId' packet length {} ",size);
                            return;
                          }
                          const PacketDeviceIdC *pkt = (const PacketDeviceIdC *) data;
                          // We can only deal with devices after they've been allocated an id.
                          if(pkt->m_deviceId == 0) {
                            // Need to do something?
                            return ;
                          }
                          std::shared_ptr<ServoC> device = DeviceEntry(pkt->m_deviceId);
                          assert(device);
                          if(device->HandlePacketAnnounce(*pkt)) {
                            ServoStatusUpdate(pkt->m_deviceId,SUT_Updated);
                          }
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
                          if(pkt->m_header.m_deviceId == 0) {
                            return ;
                          }
                          std::shared_ptr<ServoC> device = DeviceEntry(pkt->m_header.m_deviceId);
                          assert(device);
                          if(device->HandlePacketReportParam(*pkt)) {
                            ServoStatusUpdate(pkt->m_header.m_deviceId,SUT_Updated);
                          }

                        }
                       );

    while(!m_terminate)
    {
      switch(m_driverState)
      {
        case DS_Init: {
          std::this_thread::sleep_for(std::chrono::seconds(1));
        } break;
        case DS_NoConnection: {
          if(m_manageComs) {
            if(!m_coms->Open(m_deviceName.c_str())) {
              m_log->warn("Failed to open coms channel. ");
              std::this_thread::sleep_for(std::chrono::seconds(5));
              break;
            } else {
              m_driverState = DS_Connected;
            }
          } else {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if(m_coms->IsReady()) {
              m_driverState = DS_Connected;
            }
          }
        }
        // no break
        case DS_Connected: {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          if(!m_coms->IsReady()) {
            m_driverState = DS_NoConnection;
          }
        } break;
        case DS_Calibrated:
        case DS_Error:
          break;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    m_log->debug("Exiting monitor. ");
  }

  //! Set the handler for servo reports for a device.
  int DogBotAPIC::SetServoUpdateHandler(int deviceId,const std::function<void (const PacketServoReportC &report)> &handler)
  {

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
