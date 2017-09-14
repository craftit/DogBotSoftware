
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
    int m_id;
    bool m_online = false;

  };

}

#endif
