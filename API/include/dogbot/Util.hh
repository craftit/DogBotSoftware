#ifndef DOGBOG_UTIL_HEADER
#define DOGBOG_UTIL_HEADER 1

#include <math.h>

namespace DogBotN {

  //! Convert degrees to radians
  inline float Deg2Rad(float deg)
  { return deg * M_PI/ 180.0f; }

  inline float Rad2Deg(float rad)
  { return rad * 180.0f/M_PI; }

  //! Return the minimum of two values.
  template<typename ValueT>
  inline ValueT Min(ValueT v1,ValueT v2)
  { return v1 > v2 ? v2 : v1; }

  //! Return the minimum of two values.
  template<typename ValueT>
  inline ValueT Max(ValueT v1,ValueT v2)
  { return v1 > v2 ? v1 : v2; }

}

#endif
