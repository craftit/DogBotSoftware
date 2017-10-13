#ifndef DOGBOG_SERVO_HEADER
#define DOGBOG_SERVO_HEADER 1

#include "dogbot/SerialComs.hh"
#include <json/json.h>

namespace DogBotN {

  //! Motor calibration data.

  class MotorCalibrationC
  {
  public:
    MotorCalibrationC();

    //! Load configuration from JSON
    bool LoadJSON(const Json::Value &conf);

    //! Save calibration as JSON
    Json::Value SaveJSON() const;

    //! Set calibration point
    void SetCal(int place,uint16_t p1,uint16_t p2,uint16_t p3);

    //! Get calibration point
    void GetCal(int place,uint16_t &p1,uint16_t &p2,uint16_t &p3) const;

  protected:
    uint16_t m_hall[12][3];
  };

  //! Information about a single servo.

  class ServoC
  {
  public:
    ServoC();

    //! Access name of device
    const std::string &Name() const
    { return m_name; }

    //! Access the device id.
    int Id() const
    { return m_id; }

  protected:
    int32_t m_uid1 = 0;
    int m_id = -1; // Device id.
    std::string m_name;

    int32_t m_uid2 = 0;
    bool m_online = false;

  };

}

#endif
