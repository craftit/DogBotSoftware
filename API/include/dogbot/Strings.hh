#ifndef DOGBOG_STRINGS_HEADER
#define DOGBOG_STRINGS_HEADER 1

#include <cstdint>
#include "dogbot/protocol.h"

namespace DogBotN {

  //! Convert a fault code to a string
  const char *FaultCodeToString(FaultCodeT faultCode);

  //! Convert the calibration state to a string
  const char *HomedStateToString(MotionHomedStateT calibrationState);

  //! Convert the control mode to a string
  const char *ControlStateToString(ControlStateT controlState);

  //! Convert the control dynamic to a string
  const char *ControlDynamicToString(PWMControlDynamicT dynamic);

  //! Convert coms error to a string
  const char *ComsErrorTypeToString(ComsErrorTypeT errorCode);

  //! Convert coms packet type to a string
  const char *ComsPacketTypeToString(ComsPacketTypeT packetType);

  //! Convert coms device type to a string
  const char *ComsDeviceTypeToString(DeviceTypeT packetType);

  //! Convert a state change source to a string
  const char *ComsStateChangeSource(enum StateChangeSourceT changeSource);

  //! Convert coms safety mode to a string
  const char *SafetyModeToString(SafetyModeT safetyMode);

  //! Convert an parameter index to a name
  const char *ComsParameterIndexToString(ComsParameterIndexT paramIndex);

  //! Convert an parameter index to a name
  const char *ComsPositionRefrenceToString(enum PositionReferenceT referenceType);

  //! Get type information for parameters
  enum ComsParameterIndexTypeT ComsParameterIndexToType(ComsParameterIndexT paramIndex);

  //! Convert an parameter index to a name
  const char *ComsPlatformActivityToString(enum PlatformActivityT activity);


}

#endif
