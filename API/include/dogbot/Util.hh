#ifndef DOGBOG_UTIL_HEADER
#define DOGBOG_UTIL_HEADER 1

namespace DogBotN {

  //! Convert degrees to radians
  float Deg2Rad(float deg);

  //! Convert radians to degrees
  float Rad2Deg(float deg);

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
