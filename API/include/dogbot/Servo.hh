
#ifndef DOGBOG_SERVO_HEADER
#define DOGBOG_ERVO_HEADER 1

#include "SerialComs.hh"
#include <json/json.h>

namespace DogBotN {

  //! Information about a single servo.

  class ServoC
  {
  public:
    ServoC();

  protected:
    int m_id; // Servo id.
    int32_t m_uid1;
    int32_t m_uid2;
    bool m_online = false;

  };

}

#endif
