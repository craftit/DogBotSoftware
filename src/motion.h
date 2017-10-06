#ifndef MOTION_HEADER
#define MOTION_HEADER 1

#include <stdint.h>

#include "dogbot/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif
  void MotionStep(void);

  void MotionResetCalibration(void);

  void MotionUpdateEndStop(int num,bool state,float position,float velocity);

  bool MotionSetPosition(uint8_t mode,uint16_t position,uint16_t torque);

  bool MotionReport(uint16_t position,uint16_t torque,enum PositionReferenceT posRef);


  extern enum MotionCalibrationT g_motionCalibration;
  extern enum PositionReferenceT g_motionPositionReference;
  extern enum ControlStateT g_controlState;
  extern enum FaultCodeT g_lastFaultCode;
  extern bool g_indicatorState;
  extern float g_absoluteMaxTorque; // Maximum torque allowed


#ifdef __cplusplus
}
#endif


#endif
