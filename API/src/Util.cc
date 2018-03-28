
#include "dogbot/Util.hh"
#include <math.h>

namespace DogBotN {

  float Deg2Rad(float deg)
  { return deg * M_PI/ 180; }

  float Rad2Deg(float rad)
  { return rad * 180/M_PI; }

}
