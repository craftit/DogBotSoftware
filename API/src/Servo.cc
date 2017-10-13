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

  ServoC::ServoC()
  {}

}
