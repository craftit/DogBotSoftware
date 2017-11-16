
#include "dogbot/Servo.hh"
#include <string>

namespace DogBotN {


  MotorCalibrationC::MotorCalibrationC()
  {

  }

  //! Send calibration to motor
  bool MotorCalibrationC::SendCal(SerialComsC &coms,int deviceId)
  {

    return true;
  }

  //! Read calibration from motor
  bool MotorCalibrationC::ReadCal(SerialComsC &coms,int deviceId)
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
    GetJSONValue(conf,"velocityLimit",m_velocityLimit);
    GetJSONValue(conf,"currentLimit",m_currentLimit);
    GetJSONValue(conf,"positionPGain",m_positionPGain);
    GetJSONValue(conf,"velocityPGain",m_velocityPGain);
    GetJSONValue(conf,"velocityIGain",m_velocityIGain);
    GetJSONValue(conf,"motorInductance",m_motorInductance);
    GetJSONValue(conf,"motorResistance",m_motorResistance);

    Json::Value calArr = conf["encoder_cal"];
    if(!calArr.isNull()) {
      for(int i = 0;i < 12;i++) {
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
    for(int i = 0;i < 12;i++) {
      Json::Value entry;
      for(int j = 0;j < 3;j++)
        entry[j] = (int) m_hall[i][j];
      calArr[i] = entry;
    }
    Json::Value ret;
    ret["encoder_cal"] = calArr;

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

  ServoC::ServoC(const std::shared_ptr<SerialComsC> &coms,int deviceId)
   : m_id(deviceId),
     m_coms(coms)
  {
    m_name = std::to_string(deviceId);
    m_timeOfLastReport = std::chrono::steady_clock::now();
    m_timeEpoch = m_timeOfLastReport;
  }

  //! Configure from JSON
  bool ServoC::ConfigureFromJSON(const Json::Value &value)
  {

    return true;
  }

  //! Get the servo configuration as JSON
  Json::Value ServoC::ServoConfigAsJSON() const
  {
    Json::Value ret;

    ret["name"] = m_name;
    ret["controllerId"] = std::to_string(m_uid1) + "-" + std::to_string(m_uid2);

    if(m_motorCal) {
      ret["setup"] = m_motorCal->AsJSON();
    }
    return ret;
  }


  //! Process update
  bool ServoC::HandlePacketServoReport(const PacketServoReportC &report)
  {
    auto timeNow = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(m_mutexState);

    std::chrono::duration<double> timeSinceLastReport = timeNow - m_timeOfLastReport;
    if(timeSinceLastReport < m_tickRate * 200) {
      m_log->warn("Lost sync on servo {} ",m_id);
    }
    m_timeOfLastReport = timeNow;

    int tickDiff = (int) report.m_timestamp - (int) m_lastTimestamp;
    while(tickDiff < 0)
      tickDiff += 256;

    m_tick += tickDiff;

    m_position = SerialComsC::PositionReport2Angle(report.m_position);
    m_torque =  SerialComsC::TorqueReport2Value(report.m_torque);

    //m_servoRef = 0;// (enum PositionReferenceT) (pkt->m_mode & 0x3);

    return true;
  }

  //! Handle an incoming announce message.
  bool ServoC::HandlePacketAnnounce(const PacketDeviceIdC &pkt)
  {
    bool ret = false;
    {
      std::lock_guard<std::mutex> lock(m_mutexState);
      ret = (m_uid1 != pkt.m_uid[0] ||m_uid2 != pkt.m_uid[1]);
      m_uid1 = pkt.m_uid[0];
      m_uid2 = pkt.m_uid[1];
    }

    return ret;
  }
  //! Handle parameter update.
  bool ServoC::HandlePacketReportParam(const PacketParam8ByteC &pkt)
  {
    char buff[64];
    bool ret = false;

    switch ((enum ComsParameterIndexT) pkt.m_header.m_index) {
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

#if 0
    case CPI_CalibrationOffset: {
      float calAngleDeg =  (pkt.m_data.float32[0] * 360.0f / (M_PI * 2.0));
      sprintf(buff,"%f",calAngleDeg);
      displayStr += buff;
      if(pkt.m_header.m_deviceId == m_targetDeviceId) {
        emit setCalibrationAngle(calAngleDeg);
      }
    } break;
    case CPI_PWMMode: {
      enum PWMControlModeT controlMode =  (enum PWMControlModeT) pkt.m_data.uint8[0];
      if(pkt.m_header.m_deviceId == m_targetDeviceId) {
        printf("Setting control mode %d ",(int) pkt.m_data.uint8[0]);
        m_controlMode = controlMode;
        switch(controlMode) {
        case CM_Idle:     emit setControlMode("Idle"); break;
        case CM_Break:    emit setControlMode("Break"); break;
        case CM_Torque:   emit setControlMode("Torque"); break;
        case CM_Velocity: emit setControlMode("Velocity"); break;
        case CM_Position: emit setControlMode("Position"); break;
        default:
          printf("Unhandled control mode %d ",(int) pkt.m_data.uint8[0]);
        }
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

    return ret;
  }


  //! Get last reported state of the servo.
  bool ServoC::GetState(TimePointT &tick,float &position,float &velocity,float &torque) const
  {
    std::lock_guard<std::mutex> lock(m_mutexState);
    tick = m_timeEpoch + m_tick * m_tickRate;
    position = m_position;
    torque = m_torque;
    return true;
  }

  //! Update torque for the servo.
  bool ServoC::DemandTorque(float torque)
  {
    m_coms->SendTorque(m_id,torque);
    return true;
  }

  //! Demand a position for the servo
  bool ServoC::DemandPosition(float position,float torqueLimit)
  {
    m_coms->SendMoveWithEffort(m_id,position,torqueLimit,PR_Absolute);
    return true;
  }

}
