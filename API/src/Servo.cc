
#define _BSD_SOURCE 1

#include "dogbot/Servo.hh"
#include "dogbot/DogBotAPI.hh"
#include "dogbot/Util.hh"
#include "dogbot/Drv8503Registers.hh"
#include "dogbot/Strings.hh"
#include <string>
#include <cmath>

#define DODEBUG 0
#if DODEBUG
#define ONDEBUG(x) x
#else
#define ONDEBUG(x)
#endif


namespace DogBotN {

  MotorCalibrationC::MotorCalibrationC()
  {}

  //! Send calibration to motor
  bool MotorCalibrationC::SendCal(ComsC &coms,int deviceId)
  {
    return true;
  }

  //! Read calibration from motor
  bool MotorCalibrationC::ReadCal(ComsC &coms,int deviceId)
  {
    return true;
  }


  //! Get json value if set.
  bool MotorCalibrationC::GetJSONValue(const Json::Value &conf,const char *name,float &value)
  {
    Json::Value val = conf[name];
    if(!val.isDouble()) {
      return false;
    }
    value = val.asFloat();
    return true;
  }

  //! Load configuration from JSON

  bool MotorCalibrationC::LoadJSON(const Json::Value &conf)
  {
    m_motorKv = conf.get("motorKv",260.0f).asFloat();
    m_velocityLimit = conf.get("velocityLimit",1000.0f).asFloat();
    m_currentLimit = conf.get("currentLimit",10).asFloat();
    m_positionPGain = conf.get("positionPGain",1).asFloat();
    m_velocityPGain = conf.get("velocityPGain",0.1).asFloat();
    m_velocityIGain = conf.get("velocityIGain",0).asFloat();
    m_motorInductance = conf.get("motorInductance",1.0e-6).asFloat();
    m_motorResistance= conf.get("motorResistance",0).asFloat();

    Json::Value calArr = conf["encoder_cal"];
    if(!calArr.isNull()) {
      for(int i = 0;i < m_hallCalPoints;i++) {
        Json::Value entry = calArr[i];
        if(entry.isNull())
          return false;
        for(int j = 0;j < 3;j++) {
          Json::Value val = entry[j];
          if(!val.isInt()) {
            return false;
          }
          m_hall[i][j] = (int) val.asInt();
        }
      }
    }
    return true;
  }

  //! Save calibration as JSON
  Json::Value MotorCalibrationC::AsJSON() const
  {
    Json::Value calArr;
    for(int i = 0;i < m_hallCalPoints;i++) {
      Json::Value entry;
      for(int j = 0;j < 3;j++)
        entry[j] = (int) m_hall[i][j];
      calArr[i] = entry;
    }
    Json::Value ret;
    ret["encoder_cal"] = calArr;

    ret["motorKv"] = m_motorKv;
    ret["velocityLimit"] = m_velocityLimit;
    ret["currentLimit"] = m_currentLimit;
    ret["positionPGain"] = m_positionPGain;
    ret["velocityPGain"] = m_velocityPGain;
    ret["velocityIGain"] = m_velocityIGain;
    ret["motorInductance"] = m_motorInductance;
    ret["motorResistance"] = m_motorResistance;

    return ret;
  }

  //! Set calibration point
  void MotorCalibrationC::SetCal(int place,uint16_t p1,uint16_t p2,uint16_t p3)
  {
    assert(place >= 0 && place < 12);

    m_hall[place][0] = p1;
    m_hall[place][1] = p2;
    m_hall[place][2] = p3;
  }

  //! Get calibration point
  void MotorCalibrationC::GetCal(int place,uint16_t &p1,uint16_t &p2,uint16_t &p3) const
  {
    assert(place >= 0 && place < 12);

    p1 = m_hall[place][0];
    p2 = m_hall[place][1];
    p3 = m_hall[place][2];

  }

  //! --------------------------

  ServoC::ServoC(const std::shared_ptr<ComsC> &coms, int deviceId, const PacketDeviceIdC &pktAnnounce)
   : DeviceC(coms,deviceId,pktAnnounce)
  {
    m_name = m_serialNumber; // Set a default name
    Init();
  }

  ServoC::ServoC(const std::shared_ptr<ComsC> &coms, int deviceId)
   : DeviceC(coms,deviceId)
  {
    Init();
  }

  //! Type of joint
  std::string ServoC::JointType() const
  {
    return "servo";
  }

  //! Access the device type
  const char *ServoC::DeviceType() const
  {
    return "servo";
  }


  void ServoC::Init()
  {
    m_comsTimeout = std::chrono::milliseconds(500);
    m_timeOfLastReport = std::chrono::steady_clock::now();
    m_timeOfLastComs = m_timeOfLastReport;
    m_timeEpoch = m_timeOfLastReport;
    m_tickDuration = std::chrono::milliseconds(10);

    SetupConstants();

    // Things to query
    m_updateQuery.push_back(CPI_ControlState);
    m_updateQuery.push_back(CPI_FaultCode);
    m_updateQuery.push_back(CPI_SafetyMode);
    m_updateQuery.push_back(CPI_Indicator);

    m_bootloaderQueryCount = m_updateQuery.size();

    m_updateQuery.push_back(CPI_HomedState);
    m_updateQuery.push_back(CPI_PositionRef);
    m_updateQuery.push_back(CPI_ServoReportFrequency);
    m_updateQuery.push_back(CPI_PWMMode);
    m_updateQuery.push_back(CPI_CalibrationOffset);
    m_updateQuery.push_back(CPI_MotorIGain);
    m_updateQuery.push_back(CPI_VelocityPGain);
    m_updateQuery.push_back(CPI_VelocityIGain);
    m_updateQuery.push_back(CPI_VelocityLimit);
    m_updateQuery.push_back(CPI_PositionGain);
    m_updateQuery.push_back(CPI_homeIndexPosition);
    m_updateQuery.push_back(CPI_MaxCurrent);

    m_updateQuery.push_back(CPI_EndStopEnable);
    m_updateQuery.push_back(CPI_EndStopStart);
    m_updateQuery.push_back(CPI_EndStopFinal);
    m_updateQuery.push_back(CPI_SupplyVoltageScale);

  }

  //! Update coms device
  void ServoC::UpdateComs(const std::shared_ptr<ComsC> &coms)
  {
    DeviceC::UpdateComs(coms);
  }

  //! Do constant setup
  void ServoC::SetupConstants()
  {
    // See https://en.wikipedia.org/wiki/Motor_constants#Motor_Torque_constant
    m_servoKt = (60.0f * m_gearRatio) / (2 * M_PI * m_motorKv);
    //m_log->info("Setting servoKt to {} ",m_servoKt);
  }

#if 0
  void ServoC::SetUID(uint32_t uid1, uint32_t uid2)
  {
    m_uid1 = uid1;
    m_uid2 = uid2;
  }
#endif

  //! Set the name of the joint/device
  void ServoC::SetName(const std::string &name)
  {
    JointC::SetName(name);
    DeviceC::SetDeviceName(name);
  }

  //! Configure from JSON
  bool ServoC::ConfigureFromJSON(DogBotAPIC &api,const Json::Value &conf)
  {
    if(!(JointC::ConfigureFromJSON(api,conf)))
      return false;

    if(!(DeviceC::ConfigureFromJSON(api,conf)))
      return false;
    if(!m_name.empty())
      m_deviceName = m_name;
    {
      std::lock_guard<std::mutex> lock(m_mutexAdmin);
      m_motorKv = conf.get("motorKv",260.0).asFloat();
      m_gearRatio = conf.get("gearRatio",21.0).asFloat();
      m_endStopStart = conf.get("endStopStart",0).asFloat();
      m_endStopFinal = conf.get("endStopFinal",0).asFloat();
      m_endStopEnable = conf.get("endStopEnable",false).asBool();
      m_defaultHomeOffset = conf.get("homeOffset",0).asFloat();
      m_homeOffset = m_defaultHomeOffset;
      m_defaultSupplyVoltageScale = conf.get("supplyVoltageScale",1.0).asFloat();
      m_supplyVoltageScale = m_defaultSupplyVoltageScale;
    }

    Json::Value motorCal = conf["setup"];
    if(!motorCal.isNull()) {
      std::shared_ptr<MotorCalibrationC> cal = std::make_shared<MotorCalibrationC>();
      cal->LoadJSON(motorCal);
      m_motorCal = cal;
      m_motorKv = cal->MotorKv();
    }
    SetupConstants();
    return true;
  }

  //! Get the servo configuration as JSON
  void ServoC::ConfigAsJSON(Json::Value &ret) const
  {
    JointC::ConfigAsJSON(ret);
    DeviceC::ConfigAsJSON(ret);
    {
      std::lock_guard<std::mutex> lock(m_mutexAdmin);
      ret["enabled"] = m_enabled;
      ret["motorKv"] = m_motorKv;
      ret["gearRatio"] = m_gearRatio;

      ret["homeOffset"]  = m_homeOffset;
      ret["supplyVoltageScale"] = m_supplyVoltageScale;

      ret["endStopStart"] = m_endStopStart;
      ret["endStopFinal"] = m_endStopFinal;
      ret["endStopEnable"] = m_endStopEnable;

      ret["safetyMode"] = SafetyModeToString(m_safetyMode);
    }

    if(m_motorCal) {
      ret["setup"] = m_motorCal->AsJSON();
    }
  }

  bool ServoC::HandlePacketPong(const PacketPingPongC &)
  {
    std::lock_guard<std::mutex> lock(m_mutexState);
    m_timeOfLastComs = std::chrono::steady_clock::now();
    return true;
  }

  //! Process update
  bool ServoC::HandlePacketServoReport(const PacketServoReportC &report)
  {
    auto timeNow = std::chrono::steady_clock::now();

    float newPosition = ComsC::PositionReport2Angle(report.m_position);
    float newVelocity = ComsC::VelocityReport2Angle(report.m_velocity);

    //m_log->info("Velocity: {}  {} -> {} ",Name().c_str(),report.m_velocity,newVelocity);

    {
      std::lock_guard<std::mutex> lock(m_mutexState);

      std::chrono::duration<double> timeSinceLastReport = timeNow - m_timeOfLastReport;
      bool inSync = true;
      if(timeSinceLastReport > m_tickDuration * 128) {
        m_log->warn("Lost sync on servo {} ",m_id);
        inSync = false;
      }
      m_timeOfLastReport = timeNow;
      m_timeOfLastComs = timeNow;
      int tickDiff = (int) report.m_timestamp - (int) m_lastTimestamp;
      m_lastTimestamp = report.m_timestamp;
      while(tickDiff < 0)
        tickDiff += 256;
      if(tickDiff == 0)
        tickDiff = 1;

      m_tick += tickDiff;

#if 1
      m_velocity = newVelocity;
#else
      // FIXME:- Check the position reference frame.
      // Generate an estimate of the speed.
      if(inSync) {
        m_velocity = (newPosition - m_position) /  (m_tickDuration.count() * (float) tickDiff);
      } else {
        m_velocity = 0; // Set it to zero until we have up to date information.
      }
#endif
      m_positionRef = (enum PositionReferenceT) (report.m_mode & 0x3);
      m_homeIndexState = (report.m_mode & 0x8) != 0;
      m_position = newPosition;
      m_torque =  TorqueReport2Current(report.m_torque) * m_servoKt;
      m_reportedMode = report.m_mode;

      // End block, and unlock m_mutexState.
    }

    // Call things that are position reference aware
    for(auto &a : m_positionRefCallbacks.Calls()) {
      if(a) a(timeNow,m_position,m_velocity,m_torque,m_positionRef);
    }

    if(m_positionRef == PR_Absolute) {
      // Only pass absolute positions back to user code.
      for(auto &a : m_positionCallbacks.Calls()) {
        if(a) a(timeNow,m_position,m_velocity,m_torque);
      }
    }


    return true;
  }

  //! Handle parameter update.
  //! Returns true if a value has changed.

  bool ServoC::HandlePacketReportParam(const PacketParam8ByteC &pkt,int size)
  {
    char buff[64];
    bool ret = false;

    DeviceC::HandlePacketReportParam(pkt,size);

    int dataLen = size - sizeof(PacketParamHeaderC);
    if(dataLen < 0) {
      m_log->error("Parameter packet too short with length {} !",size);
      return false;
    }

    std::lock_guard<std::mutex> lock(m_mutexState);
    ComsParameterIndexT cpi = (enum ComsParameterIndexT) pkt.m_header.m_index;
    switch (cpi) {
    case CPI_FirmwareVersion: {
      if(dataLen != 1) {
        m_log->error("Firmware version has length {}, expected 1",dataLen);
        return false;
      }
      m_firmwareVersion = (unsigned) pkt.m_data.int8[0];
      if(m_firmwareVersion != DOGBOT_FIRMWARE_VERSION && m_controlState != CS_BootLoader) {
        m_log->warn("Mismatch between firmware version and API. Servo firmware is {}, API has {} ",m_firmwareVersion,DOGBOT_FIRMWARE_VERSION);
      }
    } break;
    case CPI_DriveTemp: {
      float newTemp = pkt.m_data.float32[0];
      ret = (newTemp != m_driveTemperature);
      m_driveTemperature = newTemp;
    } break;
    case CPI_MotorTemp: {
      float newTemp = pkt.m_data.float32[0];
      ret = (newTemp != m_driveTemperature);
      m_motorTemperature = newTemp;
    } break;
    case CPI_VSUPPLY: {
      float newSupplyVoltage =  ((float) pkt.m_data.uint16[0] / 1000.0f);
      ret = m_supplyVoltage != newSupplyVoltage;
      m_supplyVoltage = newSupplyVoltage;
    } break;
    case CPI_FaultCode: {
      enum FaultCodeT faultCode = (enum FaultCodeT) pkt.m_data.uint8[0];
      ret = m_faultCode != faultCode;
      if(faultCode != FC_Ok && ret) {
        m_log->warn("Fault received '{}' on device {} ",FaultCodeToString(faultCode),Name());
      }
      m_faultCode = faultCode;
    } break;
    case CPI_ControlState: {
      enum ControlStateT controlState = (enum ControlStateT) pkt.m_data.uint8[0];
      ret = m_controlState != controlState;
      if(ret) {
        // If we've changed into a control state where motor position becomes unknown.
        // reflect it in the stored state.
        switch(m_controlState)
        {
          case CS_BootLoader:
            QueryRefresh(); // We've changed from the boot loader, so we need to re-query everything
          case CS_StartUp:
          case CS_FactoryCalibrate:
          case CS_Standby:
            m_homedState = MHS_Lost;
            m_controlDynamic = CM_Off;
            m_position = 0.0;
            m_torque = 0.0;
            m_velocity = 0.0;
            break;
          default:
            break;
        }
      }
      m_controlState = controlState;
    } break;
    case CPI_HomedState: {
      enum MotionHomedStateT homedState = (enum MotionHomedStateT) pkt.m_data.uint8[0];
      ret = m_homedState != homedState;
      m_homedState = homedState;
    } break;
    case CPI_PositionGain: {
      float newGain = pkt.m_data.float32[0];
      ret = newGain != m_positionPGain;
      m_positionPGain = newGain;
    } break;
    case CPI_VelocityPGain: {
      float newVelPGain = pkt.m_data.float32[0];
      ret = newVelPGain != m_velocityPGain;
      m_velocityPGain = newVelPGain;
    } break;
    case CPI_VelocityIGain: {
      float newVelIGain = pkt.m_data.float32[0];
      ret = newVelIGain != m_velocityIGain;
      m_velocityIGain = newVelIGain;
    } break;
    case CPI_VelocityLimit: {
      float newVelLimit = pkt.m_data.float32[0];
      ret = newVelLimit != m_velocityLimit;
      m_velocityIGain = newVelLimit;
    } break;
    case CPI_MotorInductance: {
      float newVal = pkt.m_data.float32[0];
      ret = newVal != m_motorInductance;
      m_motorInductance = newVal;
    } break;
    case CPI_MotorResistance: {
      float newVal = pkt.m_data.float32[0];
      ret = newVal != m_motorResistance;
      m_motorResistance = newVal;
    } break;
    case CPI_EndStopEnable: {
      bool newVal = pkt.m_data.uint8[0] > 0;
      ret = newVal != m_endStopEnable;
      m_endStopEnable = newVal;
    } break;
    case CPI_EndStopStart: {
      float newVal = pkt.m_data.float32[0];
      ret = newVal != m_endStopStart;
      m_endStopStart = newVal;
    } break;
    case CPI_EndStopFinal: {
      float newVal = pkt.m_data.float32[0];
      ret = newVal != m_endStopFinal;
      m_endStopFinal = newVal;
    } break;
    case CPI_PWMMode: {
      enum PWMControlDynamicT controlDynamic =  (enum PWMControlDynamicT) pkt.m_data.uint8[0];
      ret = controlDynamic != m_controlDynamic;
      m_controlDynamic = controlDynamic;
    } break;
    case CPI_homeIndexPosition: {
      float newVal = pkt.m_data.float32[0];
      ret = newVal != m_homeOffset;
      m_homeOffset = newVal;
    } break;
    case CPI_USBPacketDrops: {
      m_log->error("Device {} {} USB Packet drops {} ",m_id,m_name,pkt.m_data.uint32[0]);
      ret = false;
    } break;
    case CPI_USBPacketErrors: {
      m_log->error("Device {} {} USB Packet errors {} ",m_id,m_name,pkt.m_data.uint32[0]);
      ret = false;
    } break;
    case CPI_FaultState: {
      uint32_t faultState = pkt.m_data.uint32[0];
      if(faultState != 0)
        m_log->error("Device {} {} Fault state {} ",m_id,m_name,faultState);
      ret = false;
    } break;
    case CPI_SafetyMode: {
      enum SafetyModeT safetyMode =  (enum SafetyModeT) pkt.m_data.uint8[0];
      ret = safetyMode != m_safetyMode;
      m_safetyMode = safetyMode;
    } break;
    case CPI_IndexSensor: {
      bool homeIndexState = pkt.m_data.uint8[0] != 0;
      ret = homeIndexState != m_homeIndexState;
      m_homeIndexState = homeIndexState;
    } break;
    case CPI_MaxCurrent: {
      float maxCurrent = pkt.m_data.float32[0];
      ret = (maxCurrent != m_maxCurrent);
      m_maxCurrent = maxCurrent;
    } break;
    case CPI_ServoReportFrequency: {
      float newFrequency = pkt.m_data.float32[0];
      ret = (newFrequency != m_servoReportFrequency);
      m_servoReportFrequency = newFrequency;
      if(ret) {
        m_tickDuration = std::chrono::duration<double>(1.0/m_servoReportFrequency);
      }
    } break;
    case CPI_SupplyVoltageScale: {
      float scale = pkt.m_data.float32[0];
      ret = (scale != m_supplyVoltageScale);
      m_supplyVoltageScale = scale;
    } break;
    case CPI_DRV8305_01: {
      if(dataLen != 2)
        m_log->error("Unexpected data size for packet {} of {} ",ComsParameterIndexToString(cpi),size);
      uint16_t value = pkt.m_data.uint16[0];
      std::string flagsStr;
      if(value & DRV8503_WARN_FAULT) {
        flagsStr += "FAULT,";
      }
      if(value & DRV8503_WARN_TEMP_175C) {
        flagsStr += "+175C,";
      }
      if(value & DRV8503_WARN_PVDD_UVFL) {
        flagsStr += "PVDD Under Voltage,";
      }
      if(value & DRV8503_WARN_PVDD_OVFL) {
        flagsStr += "PVDD Over Voltage,";
      }
      if(value & DRV8503_WARN_VDS_STATUS) {
        flagsStr += "VDS Over Current,";
      }
      if(value & DRV8503_WARN_VCHP_UVFL) {
        flagsStr += "Charge pump under voltage,";
      }
      if(value & DRV8503_WARN_TEMP_105C) {
        flagsStr += "+105C,";
      }
      if(value & DRV8503_WARN_TEMP_125C) {
        flagsStr += "+125C,";
      }
      if(value & DRV8503_WARN_TEMP_135C) {
        flagsStr += "+125C,";
      }
      if(value & DRV8503_WARN_OVERTEMP) {
        flagsStr += "Over temperature warning,";
      }
      m_log->error(" {} ({}) DRV8305 Reg 1, Warnings {:b} : {} ",Name(),(int)m_id,value,flagsStr);
    } break;
    case CPI_DRV8305_02: {
      if(dataLen != 2)
        m_log->error("Unexpected data size for packet {} of {} ",ComsParameterIndexToString(cpi),size);
      uint16_t value = pkt.m_data.uint16[0];
      std::string flagsStr;

      if(value & DRV8503_OVVDS_VDS_HA) {
        flagsStr += "VDA High A,";
      }
      if(value & DRV8503_OVVDS_VDS_LA) {
        flagsStr += "VDA Low A,";
      }
      if(value & DRV8503_OVVDS_VDS_HB) {
        flagsStr += "VDA High B,";
      }
      if(value & DRV8503_OVVDS_VDS_LB) {
        flagsStr += "VDA Low B,";
      }
      if(value & DRV8503_OVVDS_VDS_HC) {
        flagsStr += "VDA High C,";
      }
      if(value & DRV8503_OVVDS_VDS_LC) {
        flagsStr += "VDA Low C,";
      }
      if(value & DRV8503_OVVDS_SNS_C_OCP) {
        flagsStr += "Sense C over current,";
      }
      if(value & DRV8503_OVVDS_SNS_B_OCP) {
        flagsStr += "Sense B over current,";
      }
      if(value & DRV8503_OVVDS_SNS_A_OCP) {
        flagsStr += "Sense A over current,";
      }
      m_log->error(" {} ({}) DRV8305 Reg 2, OV/VDS Faults {:b} : {} ",Name(),(int) m_id,value,flagsStr);
    } break;
    case CPI_DRV8305_03: {
      if(dataLen != 2)
        m_log->error("Unexpected data size for packet {} of {} ",ComsParameterIndexToString(cpi),size);
      uint16_t value = pkt.m_data.uint16[0];
      std::string flagsStr;
      if(value & DRV8503_FAULTS_PVDD_UVLO2) {
        flagsStr += "PVDD Under voltage 2,";
      }
      if(value & DRV8503_FAULTS_WD_FAULT) {
        flagsStr += "Watchdog,";
      }
      if(value & DRV8503_FAULTS_OTSD) {
        flagsStr += "Over temperature,";
      }
      if(value & DRV8503_FAULTS_VREG_UV) {
        flagsStr += "VREG under voltage,";
      }
      if(value & DRV8503_FAULTS_AVDD_UVLO) {
        flagsStr += "AVDD under voltage,";
      }
      if(value & DRV8503_FAULTS_VCP_LSD_UVLO2) {
        flagsStr += "Low side gate supply,";
      }
      if(value & DRV8503_FAULTS_VCPH_UVLO2) {
        flagsStr += "High side charge pump under voltage 2,";
      }
      if(value & DRV8503_FAULTS_VCPH_OVLO) {
        flagsStr += "High side charge pump over voltage,";
      }
      if(value & DRV8503_FAULTS_VCPH_OVLO_ABS) {
        flagsStr += "High side charge pump over voltage ABS,";
      }
      m_log->error(" {} ({}) DRV8305 Reg 3, Faults {:b} : {} ",Name(),(int) m_id,value,flagsStr);
    } break;
    case CPI_DRV8305_04: {
      if(dataLen != 2)
        m_log->error("Unexpected data size for packet {} of {} ",ComsParameterIndexToString(cpi),size);
      uint16_t value = pkt.m_data.uint16[0];
      std::string flagsStr;

      if(value & DRV8503_VGSFAULTS_VGS_HA) {
        flagsStr += "VGS high side gate A,";
      }
      if(value & DRV8503_VGSFAULTS_VGS_LA) {
        flagsStr += "VGS low side gate A,";
      }
      if(value & DRV8503_VGSFAULTS_VGS_HB) {
        flagsStr += "VGS high side gate B,";
      }
      if(value & DRV8503_VGSFAULTS_VGS_LB) {
        flagsStr += "VGS low side gate B,";
      }
      if(value & DRV8503_VGSFAULTS_VGS_HC) {
        flagsStr += "VGS high side gate C,";
      }
      if(value & DRV8503_VGSFAULTS_VGS_LC) {
        flagsStr += "VGS low side gate C,";
      }
      m_log->error(" {} ({}) DRV8305 Reg 4, VGS Faults {:b} : {} ",Name(),(int) m_id,value,flagsStr);
    }
      break;
    default:
      break;
    }

    for(auto &a : m_parameterCallbacks.Calls()) {
      if(a) a(cpi);
    }


    return ret;
  }


  //! Get last reported state of the servo.
  bool ServoC::GetState(TimePointT &tick,double &position,double &velocity,double &torque) const
  {
    std::lock_guard<std::mutex> lock(m_mutexState);
    if(m_positionRef != PR_Absolute)
      return false;
    tick = m_timeEpoch + m_tick * m_tickDuration;
    position = m_position;
    torque = m_torque;
    velocity = m_velocity;
    return true;
  }

  //! Get raw state information
  bool ServoC::GetRawState(TimePointT &tick,double &position,double &velocity,double &torque,enum PositionReferenceT &posRef)
  {
    std::lock_guard<std::mutex> lock(m_mutexState);
    posRef = m_positionRef;
    position = m_position;
    velocity = m_velocity;
    torque = m_torque;
    tick = m_timeEpoch + m_tick * m_tickDuration;
    return true;
  }


  //! Estimate state at the given time.
  bool ServoC::GetStateAt(TimePointT theTime,double &position,double &velocity,double &torque) const
  {
    std::lock_guard<std::mutex> lock(m_mutexState);
    if(m_positionRef != PR_Absolute)
      return false;
    TimePointT lastTick = m_timeEpoch + m_tick * m_tickDuration;
    auto timeDiff = theTime - lastTick;
    if(fabs(timeDiff.count()) < m_tickDuration.count() * 5) {
      // Correct position for current speed.
      position = m_position + m_velocity * timeDiff.count();
    } else {
      // Out of date, just use last reported position.
      // This will 'pop' back to the last reported position, not ideal.
      position = m_position;
    }
    // Assume torque and velocity are approximately constant.
    torque = m_torque;
    velocity = m_velocity;
    return true;
  }

  bool ServoC::UpdateTick(TimePointT timeNow)
  {
    DeviceC::UpdateTick(timeNow);

    bool ret = false;
    std::chrono::duration<double> timeSinceLastReport;
    FaultCodeT faultCode = FC_Unknown;
    {
      std::lock_guard<std::mutex> lock(m_mutexState);
      timeSinceLastReport = timeNow - m_timeOfLastComs;
      faultCode = m_faultCode;
    }

    std::chrono::duration<double> comsTimeout = m_comsTimeout;
    switch(m_controlState)
    {
    case CS_Ready:
    case CS_Diagnostic:
      comsTimeout = m_comsTimeout;
    break;
    case CS_FactoryCalibrate:
      comsTimeout = std::chrono::seconds(30);
    break;
    default:
      comsTimeout = std::chrono::seconds(2);
    }

    if(faultCode != FC_Unknown) {
      if(timeSinceLastReport > comsTimeout) {
        m_faultCode = FC_Unknown;

#if 0
        // These are useful to see, as knowing the last known state for the driver can help diagnose faults.
        // Change everything to default.
        m_homedState = MHS_Lost;
        m_positionRef = PR_Relative;
        m_controlDynamic = CM_Fault;
        m_driveTemperature = 0.0;
        m_motorTemperature = 0.0;
        m_supplyVoltage = 0.0;
#endif

        ret = true;
        m_log->warn("Lost contact with servo {}  for {} seconds",m_id,timeSinceLastReport.count());
        m_toQuery = -1;
        m_firmwareVersion = -1;

        // Set velocity estimate to zero.
        m_velocity = 0;
      }
    } else {
#if 1
      if(timeSinceLastReport < comsTimeout) {
        // Re-query servo status.
        if((m_faultCode == FC_Unknown) &&
            (m_toQuery == (int) m_updateQuery.size()))
        {
          m_toQuery = -1;
          m_firmwareVersion = -1;
          m_log->warn("Regained contact with servo {}, querying.",m_id);
        }
      }
#endif
    }

    // Go through updating things, and avoiding flooding the bus.
    if(m_toQuery < (int) m_updateQuery.size() && m_coms && m_coms->IsReady()) {
      // Don't query everything in boot-loader mode.
      if(m_toQuery < m_bootloaderQueryCount) {
        if(m_controlState == CS_BootLoader) {
          TimePointT now = TimePointT::clock::now();
          if((now - m_lastVersionQuery) > std::chrono::seconds(1)) {
            m_coms->SendQueryParam(m_id,CPI_ControlState);
          }
        } else {
          // Don't proceed until we have a firmware version number
          if(m_firmwareVersion < 0) {
            TimePointT now = TimePointT::clock::now();
            if(m_toQuery < 0 || (now - m_lastVersionQuery) > std::chrono::seconds(2) ) {
              // Do we need a timeout / retry ?
              m_coms->SendQueryParam(m_id,CPI_ControlState);
              m_coms->SendQueryParam(m_id,CPI_FirmwareVersion);
              m_lastVersionQuery = now;
              m_toQuery = 0;
            }
          } else {
            if(m_toQuery < 0) m_toQuery = 0;
            m_coms->SendQueryParam(m_id,m_updateQuery[m_toQuery]);
            m_toQuery++;
          }
        }
      }
    }
    return ret;
  }


  //! Update torque for the servo.
  bool ServoC::DemandTorque(float torque)
  {
    if(!m_enabled || !IsFirmwareVersionOk())
      return false;
    float current = torque / (m_maxCurrent* m_servoKt);
    m_coms->SendTorque(m_id,current);
    return true;
  }

  //! Demand a position for the servo, torque limit is in Newton-meters
  bool ServoC::DemandPosition(float position,float torqueLimit,enum PositionReferenceT positionRef)
  {
    if(!m_enabled || !IsFirmwareVersionOk())
      return false;
    float currentLimit = torqueLimit / (m_maxCurrent* m_servoKt);
    m_coms->SendMoveWithEffortLimit(m_id,position,currentLimit,positionRef);
    return true;
  }

  //! Demand a position for the servo
  bool ServoC::DemandPosition(float position,float torqueLimit)
  {
    if(!m_enabled || !IsFirmwareVersionOk())
      return false;
    if(m_positionRef != PR_Absolute) {
      m_log->warn("Joint not yet homed, ignoring move request. ");
      return false;
    }
    JointC::DemandPosition(position,torqueLimit);
    return DemandPosition(position,torqueLimit,m_positionRef);
  }

  //! Set the trajectory update rate in Hz.
  bool ServoC::SetupTrajectory(float period,float torqueLimit)
  {
    if(!m_enabled || !IsFirmwareVersionOk())
      return false;
    JointC::SetupTrajectory(period,torqueLimit);
    if(!m_coms->SetParam(m_id,CPI_MotionUpdatePeriod,period))
      return false;
    float effortLimit = torqueLimit / m_servoKt;
    if(!m_coms->SetParam(m_id,CPI_CurrentLimit,effortLimit))
      return false;
    return true;
  }

  //! Demand next position for the servo, with expected torque
  bool ServoC::DemandTrajectory(float position,float torque)
  {
    if(!m_enabled) {
      m_log->warn("Servo {} not enabled, DemandTrajectory  dropped.",Name());
      return false;
    }
    if(!IsFirmwareVersionOk()) {
      if(!IsPresent()) {
        m_log->warn("Servo {} not present, DemandTrajectory  dropped.",Name());
        return false;
      }
      m_log->warn("Servo {} firmware mismatch, DemandTrajectory  dropped.",Name());
      return false;
    }
    if(m_positionRef != PR_Absolute) {
      m_log->warn("Servo {} not yet homed, ignoring trajectory request. ",Name());
      return false;
    }
    float effort = torque / (m_maxCurrent* m_servoKt);
    JointC::DemandTrajectory(position,torque);
    m_coms->SendMoveWithEffort(m_id,position,effort,PR_Absolute,m_trajectoryTimestamp);
    m_trajectoryTimestamp++;
    return true;
  }


  void ServoC::QueryRefresh()
  {
    m_queryCycle = -1;
  }

  class HomeStateC
  {
  public:

    float m_indexPositions[4] = { nanf(""),nanf(""),nanf(""),nanf("") };
    float m_indexOffsets[4] = { nanf(""),nanf(""),nanf(""),nanf("") };

    float m_minIndexBound = Deg2Rad(-360);
    float m_maxIndexBound = Deg2Rad(360);

    float m_torqueLimit = 1.0;
    float m_timeOut = 2.0;
    float m_currentPosition = 0;

    const float m_indexAngleWidth = Deg2Rad(28); // This is a conservative estimate, it is actually a little less.
    const float m_hysterisisWidth = Deg2Rad(5);

    float m_minRange = -Deg2Rad(-360);
    float m_maxRange = +Deg2Rad( 360);

    float m_homeOffset = nanf("");
    float m_homeOffsetError = Deg2Rad(360); // Could be anywhere

    //! Called if the index position is active.
    void InitialPosition(float position,bool indexActive)
    {
      if(indexActive) {
        m_minIndexBound = position - Deg2Rad(180);
        m_maxIndexBound = position + Deg2Rad(180);
      } else {
        m_minIndexBound = position - m_indexAngleWidth;
        m_maxIndexBound = position + m_indexAngleWidth;
        m_homeOffset = position;
        m_homeOffsetError = m_indexAngleWidth;
      }
    }

    static int HomeIndexUpdateOffset(bool newState,bool velocityPositive)
    {
      return (newState ? 1 : 0) + ((velocityPositive ? 2 : 0));
    }

    void IndexStateChange(bool newIndexState,float position,float velocity)
    {
      int offset = HomeIndexUpdateOffset(newIndexState,velocity > 0);
      m_indexPositions[offset] = position;

      float sc = 0;
      float ss = 0;
      float count = 0;
      // Update home position estimate.
      for(int i = 0;i < 4;i++) {
        if(std::isnan(m_indexPositions[i]))
          continue;
        sc += cos(m_indexPositions[i]-m_indexOffsets[i]);
        ss += sin(m_indexPositions[i]-m_indexOffsets[i]);
        count++;
      }
      m_homeOffset = atan2(ss,sc);
      // FIXME:- Compute the actual noise on the position.
      m_homeOffsetError = Deg2Rad(8.0/count); // Hopefully less than this.
    }

    void InitOffsets()
    {
      // Build table of estimated transition positions.
      m_indexOffsets[HomeIndexUpdateOffset(false,false)] =  m_indexAngleWidth/2.0 - m_hysterisisWidth;
      m_indexOffsets[HomeIndexUpdateOffset(false,true)]  = -m_indexAngleWidth/2.0 + m_hysterisisWidth;;
      m_indexOffsets[HomeIndexUpdateOffset(true,false)]  = -m_indexAngleWidth/2.0;
      m_indexOffsets[HomeIndexUpdateOffset(true,true)]   =  m_indexAngleWidth/2.0;
    }

    HomeStateC()
    {
      InitOffsets();
    }

};

  //! Home a point position. This will block until homing is complete.

  bool ServoC::HomeJoint(bool restorePosition,HomeDirectionHintT directionHint)
  {
    //m_log->info("HomeJoint called.");
    if(!IsFirmwareVersionOk()) {
      m_log->info("Joint {} firmware version mismatch.",Name());
      return false;
    }
    if(!IsEnabled()) {
      m_log->info("Joint {} disabled.",Name());
      return false;
    }
    if(m_homedState == MHS_Homed) {
      m_log->info("Joint {} already homed.",Name());
      return true;
    }

    // Is controller ready ?
    if(m_controlState != CS_Ready) {
      m_log->error("Can't home {}, joint not in ready state.",Name());
      return false;
    }

    // Set joint velocity limit to something nice and slow.
    if(!m_coms->SetParam(m_id,CPI_VelocityLimit,100.0f)) {
      m_log->error("Can't home {}, failed to set velocity limit.",Name());
      return false;
    }

    // Put joint into position mode.
    if(!m_coms->SetParam(m_id,CPI_PWMMode,(uint8_t) CM_Position)) {
      m_log->error("Can't home {}, failed to set control mode to position.",Name());
      return false;
    }

    TimePointT tick;
    float position,torque,velocity;
    PositionReferenceT positionRef;
    bool homeIndexState;
    {
      std::lock_guard<std::mutex> lock(m_mutexState);
      tick = m_timeEpoch + m_tick * m_tickDuration;
      position = m_position;
      torque = m_torque;
      velocity = m_velocity;
      positionRef = m_positionRef;
      homeIndexState = m_homeIndexState;
    }

    float torqueLimit = 3; // Limit on torque to use while homing
    float timeOut = 40.0;
    int maxCycles = 5;
    HomeStateC homeState;

    // Already homed?
    if(positionRef == PR_Absolute) {
      m_log->info("Joint {} is reporting positions in absolute coordinates, it must be already homed.",Name());
      return true;
    }

    // If the index sensor is already active then we have some idea where we are.
    homeState.InitialPosition(position,homeIndexState);

    m_log->info("Initial position {}, bounds from {} to {} ",Rad2Deg(position),Rad2Deg(homeState.m_minIndexBound),Rad2Deg(homeState.m_maxIndexBound));

    float changedAt = 0;
    float targetPosition = 0; //homeState.m_maxIndexBound;
    bool positiveVelocity = false; //targetPosition > position;

    switch(directionHint)
    {
      case HDH_None:
      case HDH_Anticlockwise:
        if(homeState.m_maxIndexBound > position) {
          targetPosition = homeState.m_maxIndexBound;
          positiveVelocity = true;
        } else {
          targetPosition = homeState.m_minIndexBound;
          positiveVelocity = false;
        }
        break;
      case HDH_Clockwise:
        if(homeState.m_minIndexBound < position) {
          targetPosition = homeState.m_minIndexBound;
          positiveVelocity = false;
        } else {
          targetPosition = homeState.m_maxIndexBound;
          positiveVelocity = true;
        }
        break;
    }

    DemandPosition(targetPosition,torqueLimit,PR_Relative);
    TimePointT startTime = TimePointT::clock::now();

    bool indexStateChanged = false;
    bool currentIndexState = homeIndexState;
    std::timed_mutex done;
    done.lock();

    int cycles = 0;
    bool homed = false;


    CallbackHandleC cb = AddPositionRefUpdateCallback(
        [this,torqueLimit,&done,&homed,&homeState,&currentIndexState,&targetPosition,&positiveVelocity,&cycles,&startTime,maxCycles]
         (TimePointT theTime,double position,double velocity,double torque,enum PositionReferenceT positionRef)
        {
          if(cycles > maxCycles) { // Have we aborted?
            return ;
          }
          if(positionRef != PR_Relative || m_homedState == MHS_Homed) {
            homed = true;
            done.unlock();
            return ;
          }
          if(currentIndexState != m_homeIndexState) {
            currentIndexState = m_homeIndexState;
            homeState.IndexStateChange(currentIndexState,position,velocity);

            // If it is true, it is time to change directions.
            if(currentIndexState) {
              cycles++;
              if(cycles > maxCycles) {
                m_log->warn("Too many homing cycles, aborting.");
                done.unlock();
                return ;
              }
              if(positiveVelocity) {
                targetPosition = position - homeState.m_indexAngleWidth;
                positiveVelocity = false;
                DemandPosition(targetPosition,torqueLimit,PR_Relative);
                startTime = TimePointT::clock::now();
              } else {
                targetPosition = position + homeState.m_indexAngleWidth;
                positiveVelocity = true;
                DemandPosition(targetPosition,torqueLimit,PR_Relative);
                startTime = TimePointT::clock::now();
              }
              // Done for the moment.
              return ;
            }
          }

          // Stalled ?
          bool doReverse = false;
          double timeSinceStart =  (theTime - startTime).count();
          if(fabs(velocity) < 2.0 &&
              fabs(torque) >= (torqueLimit * 0.95) &&
              timeSinceStart > 0.5)
          {
            m_log->info("Stalled at {} target {} Torque: {} Time: {}. ",Rad2Deg(position),Rad2Deg(targetPosition),torque,timeSinceStart);
            doReverse = true;
          } else {
            // Have we reach our destination ?
            if(fabs(position - targetPosition) < (M_PI/64.0)) {
              m_log->info("Got to position {} . ",Rad2Deg(targetPosition));
              doReverse = true;
            }
          }

          // At target position ?
          if(doReverse) {
            // Turn around and go the other way.

            cycles++;
            if(cycles > maxCycles) {
              m_log->warn("Too many homing cycles, aborting.");
              done.unlock();
              return ;
            }
            if(positiveVelocity) {
              targetPosition = homeState.m_minIndexBound;
              m_log->info("Reverse (-ve) to position {} . ",Rad2Deg(targetPosition));
              positiveVelocity = false;
              DemandPosition(targetPosition,torqueLimit,PR_Relative);
              startTime = TimePointT::clock::now();
            } else {
              targetPosition = homeState.m_maxIndexBound;
              m_log->info("Reverse (+ve) to position {} . ",Rad2Deg(targetPosition));
              positiveVelocity = true;
              DemandPosition(targetPosition,torqueLimit,PR_Relative);
              startTime = TimePointT::clock::now();
            }

            return ;
          }

        }
    );

    using Ms = std::chrono::milliseconds;

    if(!done.try_lock_for(Ms((int) (1000 * timeOut)))) {
      m_log->warn("Timed out going to position. ");
      //return false;
    }
    cb.Remove();
    if(restorePosition) {
      // Got back to where we started.
      m_log->info("Returning to original position {}. ",Rad2Deg(position));
      DemandPosition(position,torqueLimit,PR_Relative);
    }

    return homed;
  }

  //! Move joint until we see an index state change.
  JointMoveStatusT ServoC::MoveUntilIndexChange(
      float targetPosition,
      float torqueLimit,
      bool currentIndexState,
      float &changedAt,
      bool &indexChanged,
      double timeOut
      )
  {
    if(!IsFirmwareVersionOk()) {
      m_log->info("Joint {} firmware version mismatch.",Name());
      return JMS_Error;
    }
    if(!IsEnabled()) {
      return JMS_IncorrectMode;
    }
    if(m_controlState != CS_Ready) {
      m_log->error("Move until index change for {} failed, joint not in ready state.",Name());
      return JMS_IncorrectMode;
    }
    std::timed_mutex done;
    done.lock();

    TimePointT startTime = TimePointT::clock::now();
    if(!DemandPosition(targetPosition,torqueLimit,PR_Relative))
      return JMS_Error;

    JointMoveStatusT ret = JMS_Error;
    indexChanged = false;

    CallbackHandleC cb = AddPositionRefUpdateCallback(
        [this,targetPosition,torqueLimit,currentIndexState,&ret,&done,startTime,&changedAt,&indexChanged]
         (TimePointT theTime,double position,double velocity,double torque,enum PositionReferenceT positionRef)
        {
          if(positionRef != PR_Relative) {
            m_log->info("Not in relative mode. ");
            ret = JMS_IncorrectMode;
            done.unlock();
            return ;
          }
          if(currentIndexState != m_homeIndexState) {
            ret = JMS_Done;
            indexChanged = true;
            changedAt = position;
            done.unlock();
            return ;
          }
          // At target position ?
          if(fabs(position - targetPosition) < (M_PI/64.0)) {
            m_log->info("Got to position {} . ",targetPosition);
            ret = JMS_Done;
            done.unlock();
            return ;
          }
          // Stalled ?
          double timeSinceStart =  (theTime - startTime).count();
          if(fabs(velocity) < (M_PI/64.0) && torque >= (torqueLimit * 0.95) && timeSinceStart > 0.5){
            m_log->info("Stalled at {}. ",position);
            ret = JMS_Stalled;
            done.unlock();
            return ;
          }

        }
    );
    using Ms = std::chrono::milliseconds;

    if(!done.try_lock_for(Ms((int) (1000 * timeOut)))) {
      m_log->warn("Timed out going to position. ");
      return JMS_TimeOut;
    }
    cb.Remove();
    return ret;
  }

  //! Restore configuration from stored settings.
  bool ServoC::RestoreConfig()
  {
    m_log->info("Restoring configuration for device {}  SupplyVoltageScale= {} ",Name(),m_defaultSupplyVoltageScale);
    if(!DeviceC::RestoreConfig())
      return false;
    if(m_controlState != CS_Standby) {
      m_log->warn("Can't restore state unless controller is in standby.");
      return false;
    }
    if(!m_coms->SetParam(m_id,CPI_homeIndexPosition,m_defaultHomeOffset)) {
      m_log->error("Failed to set homeIndexPostion for device {} ",m_deviceName);
      return false;
    }
#if 0
    if(!m_coms->SetParam(m_id,CPI_SupplyVoltageScale,m_defaultSupplyVoltageScale)) {
      m_log->error("Failed to set supply voltage scale for device {} ",m_deviceName);
      return false;
    }
#endif
    m_coms->SendStoreConfig(m_id);
    return true;
  }

  bool ServoC::IsFirmwareVersionOk() const
  {
    return m_firmwareVersion == DOGBOT_FIRMWARE_VERSION;
  }

}
