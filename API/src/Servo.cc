
#include "dogbot/Servo.hh"
#include <string>

namespace DogBotN {


  MotorCalibrationC::MotorCalibrationC()
  {

  }

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
   : m_uid1(pktAnnounce.m_uid[0]),
     m_uid2(pktAnnounce.m_uid[1]),
     m_id(deviceId),
     m_coms(coms)
  {
    m_name = std::to_string(pktAnnounce.m_uid[0]) + "-" + std::to_string(pktAnnounce.m_uid[1]);
    Init();
  }

  ServoC::ServoC(const std::shared_ptr<ComsC> &coms, int deviceId)
   : m_id(deviceId),
     m_coms(coms)
  {
    if(deviceId != 0)
      m_name = std::to_string(deviceId);
    Init();
  }

  //! Type of joint
  std::string ServoC::JointType() const
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
    m_updateQuery.push_back(CPI_HomedState);
    m_updateQuery.push_back(CPI_PositionRef);
    m_updateQuery.push_back(CPI_PWMMode);
    m_updateQuery.push_back(CPI_CalibrationOffset);
    m_updateQuery.push_back(CPI_OtherJoint);
    m_updateQuery.push_back(CPI_Indicator);
    m_updateQuery.push_back(CPI_OtherJointOffset);
    m_updateQuery.push_back(CPI_OtherJointGain);
    m_updateQuery.push_back(CPI_MotorIGain);
    m_updateQuery.push_back(CPI_VelocityPGain);
    m_updateQuery.push_back(CPI_VelocityIGain);
    m_updateQuery.push_back(CPI_VelocityLimit);
    m_updateQuery.push_back(CPI_PositionGain);
    m_updateQuery.push_back(CPI_homeIndexPosition);
    m_updateQuery.push_back(CPI_MaxCurrent);

    // See https://en.wikipedia.org/wiki/Motor_constants#Motor_Torque_constant

  }

  //! Update coms device
  void ServoC::UpdateComs(const std::shared_ptr<ComsC> &coms)
  {
    JointC::UpdateComs(coms);
    m_coms = coms;
  }

  //! Do constant setup
  void ServoC::SetupConstants()
  {
    m_servoKt = (60.0f * m_gearRatio) / (2 * M_PI * m_motorKv);
  }



  void ServoC::SetUID(uint32_t uid1, uint32_t uid2)
  {
    m_uid1 = uid1;
    m_uid2 = uid2;
  }

  //! Configure from JSON
  bool ServoC::ConfigureFromJSON(DogBotAPIC &api,const Json::Value &conf)
  {
    if(!(JointC::ConfigureFromJSON(api,conf)))
      return false;

    {
      std::lock_guard<std::mutex> lock(m_mutexAdmin);
      m_uid1 = conf.get("uid1",-1).asInt();
      m_uid2 = conf.get("uid2",-1).asInt();
      m_enabled = conf.get("enabled",false).asBool();
      m_motorKv = conf.get("motorKv",260.0).asFloat();
      m_gearRatio = conf.get("gearRatio",21.0).asFloat();
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
  Json::Value ServoC::ConfigAsJSON() const
  {
    Json::Value ret = JointC::ConfigAsJSON();

    {
      std::lock_guard<std::mutex> lock(m_mutexAdmin);
      ret["uid1"] = m_uid1;
      ret["uid2"] = m_uid2;
      ret["deviceId"] = m_id;
      ret["enabled"] = m_enabled;
      ret["motorKv"] = m_motorKv;
      ret["gearRatio"] = m_gearRatio;
    }

    if(m_motorCal) {
      ret["setup"] = m_motorCal->AsJSON();
    }
    return ret;
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

      float newPosition = ComsC::PositionReport2Angle(report.m_position);

      // FIXME:- Check the position reference frame.

      // Generate an estimate of the speed.
      if(inSync) {
        m_velocity = (newPosition - m_position) /  (m_tickDuration.count() * (float) tickDiff);
      } else {
        m_velocity = 0; // Set it to zero until we have up to date information.
      }
      m_positionRef = (enum PositionReferenceT) (report.m_mode & 0x3);
      m_position = newPosition;
      m_torque =  ComsC::TorqueReport2Current(report.m_torque) * m_servoKt;
      m_reportedMode = report.m_mode;

      // End block, and unlock m_mutexState.
    }

    for(auto &a : m_positionCallbacks.Calls()) {
      if(a) a(timeNow,m_position,m_velocity,m_torque);
    }


    return true;
  }

  //! Handle an incoming announce message.
  bool ServoC::HandlePacketAnnounce(const PacketDeviceIdC &pkt,bool isMaster)
  {
    bool ret = false;
    if(pkt.m_deviceId != m_id && isMaster) {
      m_coms->SendSetDeviceId(m_id,m_uid1,m_uid2);
      ret = true;
    }
    std::lock_guard<std::mutex> lock(m_mutexState);
    auto timeNow = std::chrono::steady_clock::now();
    m_timeOfLastComs = timeNow;
    return ret;
  }

  //! Handle parameter update.
  bool ServoC::HandlePacketReportParam(const PacketParam8ByteC &pkt)
  {
    char buff[64];
    bool ret = false;

    std::lock_guard<std::mutex> lock(m_mutexState);
    auto timeNow = std::chrono::steady_clock::now();
    m_timeOfLastComs = timeNow;
    ComsParameterIndexT cpi = (enum ComsParameterIndexT) pkt.m_header.m_index;
    switch (cpi) {
    case CPI_DriveTemp: {
      float newTemp = pkt.m_data.float32[0];
      ret = (newTemp != m_temperature);
      m_temperature = newTemp;
    } break;
    case CPI_VSUPPLY: {
      float newSupplyVoltage =  ((float) pkt.m_data.uint16[0] / 1000.0f);
      ret = m_supplyVoltage != newSupplyVoltage;
      m_supplyVoltage = newSupplyVoltage;
    } break;
    case CPI_FaultCode: {
      enum FaultCodeT faultCode = (enum FaultCodeT) pkt.m_data.uint8[0];
      ret = m_faultCode != faultCode;
      m_faultCode = faultCode;
    } break;
    case CPI_ControlState: {
      enum ControlStateT controlState = (enum ControlStateT) pkt.m_data.uint8[0];
      ret = m_controlState != controlState;
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
    case CPI_PWMMode: {
      enum PWMControlDynamicT controlDynamic =  (enum PWMControlDynamicT) pkt.m_data.uint8[0];
      ret = controlDynamic != m_controlDynamic;
      m_controlDynamic = controlDynamic;
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
      m_log->error("Device {} {} Fault state {} ",m_id,m_name,pkt.m_data.uint32[0]);
      ret = false;
    } break;
#if 0
    case CPI_CalibrationOffset: {
      float calAngleDeg =  (pkt.m_data.float32[0] * 360.0f / (M_PI * 2.0));
      sprintf(buff,"%f",calAngleDeg);
      displayStr += buff;
      if(pkt.m_header.m_deviceId == m_targetDeviceId) {
        emit setCalibrationAngle(calAngleDeg);
      }
    } break;
    case CPI_OtherJoint: {
      if(pkt.m_header.m_deviceId == m_targetDeviceId) {
        uint8_t otherJointId = pkt.m_data.uint8[0];
        setOtherJoint(otherJointId);
      }
    } break;
    case CPI_PositionRef: {
      if(pkt.m_header.m_deviceId == m_targetDeviceId) {
        enum PositionReferenceT posRef = (enum PositionReferenceT) pkt.m_data.uint8[0];
        switch(posRef)
        {
          case PR_Relative: emit setPositionRef("Relative"); break;
          case PR_Absolute: emit setPositionRef("Absolute"); break;
          case PR_OtherJointRelative: emit setPositionRef("RelativeOther"); break;
          case PR_OtherJointAbsolute: emit setPositionRef("AbsoluteOther"); break;
        }
      }
    } break;
    case CPI_Indicator: {
      if(pkt.m_header.m_deviceId == m_targetDeviceId) {
        emit setIndicator(pkt.m_data.uint8[0] > 0);
      }
    } break;
    case CPI_OtherJointGain:
      if(pkt.m_header.m_deviceId == m_targetDeviceId) {
        emit setOtherJointGain(pkt.m_data.float32[0]);
      }
      break;
    case CPI_OtherJointOffset:
      if(pkt.m_header.m_deviceId == m_targetDeviceId) {
        emit setOtherJointOffset(pkt.m_data.float32[0] * 360.0 / (2.0 * M_PI));
      }
      break;
    case CPI_MotorInductance:
      sprintf(buff,"\n Inductance: %f ", pkt.m_data.float32[0]);
      displayStr += buff;
      break;
    case CPI_MotorResistance:
      sprintf(buff,"\n Resistance: %f ",pkt.m_data.float32[0]);
      displayStr += buff;
      break;
    case CPI_MotorIGain:
      if(pkt.m_header.m_deviceId == m_targetDeviceId) {
        emit setMotorIGain(pkt.m_data.float32[0]);
      }
      sprintf(buff,"\n IGain: %f ",pkt.m_data.float32[0]);
      displayStr += buff;
      break;
    case CPI_MotorPGain:
      sprintf(buff,"\n PGain: %f ",pkt.m_data.float32[0]);
      displayStr += buff;
      break;
    case CPI_PhaseVelocity:
      if(pkt.m_header.m_deviceId == m_targetDeviceId) {
        emit setMotorVelocity(pkt.m_data.float32[0]);
      }
      sprintf(buff,"\n Velocity: %f ",pkt.m_data.float32[0]);
      displayStr += buff;
      ret = false;
      break;
    case CPI_DemandPhaseVelocity:
      if(pkt.m_header.m_deviceId == m_targetDeviceId) {
        emit setDemandPhaseVelocity(pkt.m_data.float32[0]);
      }
      sprintf(buff,"\n DemandVelocity: %f ",pkt.m_data.float32[0]);
      displayStr += buff;
      ret = false;
      break;
    case CPI_DRV8305_01:
    case CPI_DRV8305_02:
    case CPI_DRV8305_03:
    case CPI_DRV8305_04:
    case CPI_DRV8305_05: {
      int reg = (pkt.m_header.m_index - (int) CPI_DRV8305_01)+1;
      sprintf(buff,"\n Reg %d contents: %04X ",reg,(int) pkt.m_data.uint16[0]);
      displayStr += buff;

     } break;
#endif
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
    tick = m_timeEpoch + m_tick * m_tickDuration;
    position = m_position;
    torque = m_torque;
    velocity = m_velocity;
    return true;
  }

  //! Estimate state at the given time.
  bool ServoC::GetStateAt(TimePointT theTime,double &position,double &velocity,double &torque) const
  {
    std::lock_guard<std::mutex> lock(m_mutexState);
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
    bool ret = false;
    std::chrono::duration<double> timeSinceLastReport;
    FaultCodeT faultCode = FC_Unknown;
    {
      std::lock_guard<std::mutex> lock(m_mutexState);
      timeSinceLastReport = timeNow - m_timeOfLastComs;
      faultCode = m_faultCode;
    }
    if(faultCode != FC_Unknown) {
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

      if(timeSinceLastReport > comsTimeout) {
        m_faultCode = FC_Unknown;
        ret = true;
        m_log->warn("Lost contact with servo {}  for {} seconds",m_id,timeSinceLastReport.count());

        // Set velocity estimate to zero.
        m_velocity = 0;
      }
    }

    // Go through updating things, and avoiding flooding the bus.
    if(m_toQuery < (int) m_updateQuery.size() && m_coms && m_coms->IsReady()) {
      m_coms->SendQueryParam(m_id,m_updateQuery[m_toQuery]);
      m_toQuery++;
    }

    return ret;
  }


  //! Update torque for the servo.
  bool ServoC::DemandTorque(float torque)
  {
    float current = torque / m_servoKt;
    m_coms->SendTorque(m_id,current);
    return true;
  }

  //! Demand a position for the servo
  bool ServoC::DemandPosition(float position,float torqueLimit)
  {
    float currentLimit = torqueLimit / m_servoKt;
    m_coms->SendMoveWithEffort(m_id,position,currentLimit,m_positionRef);
    return true;
  }

  void ServoC::QueryRefresh()
  {
    m_queryCycle = 0;
  }


}
