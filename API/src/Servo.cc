#include "dogbot/Servo.hh"

namespace DogBotN {


  MotorCalibrationC::MotorCalibrationC()
  {

  }

  //! Load configuration from JSON

  bool MotorCalibrationC::LoadJSON(const Json::Value &conf)
  {
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
  Json::Value MotorCalibrationC::SaveJSON() const
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
    m_timeOfLastReport = std::chrono::steady_clock::now();
    m_timeEpoch = m_timeOfLastReport;
  }

  //! Process update
  bool ServoC::ProcessServoReport(const PacketServoReportC &report)
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
