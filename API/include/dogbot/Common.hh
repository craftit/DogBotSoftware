#ifndef DOGBOT_COMMON_HEADER
#define DOGBOT_COMMON_HEADER 1

#include <chrono>

namespace DogBotN {

  class DogBotAPIC;
  class ComsC;

  typedef std::chrono::time_point<std::chrono::steady_clock,std::chrono::duration< double > > TimePointT;
  typedef std::chrono::duration< double > DurationT;
}

#endif
