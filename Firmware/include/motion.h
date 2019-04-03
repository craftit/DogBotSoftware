#ifndef MOTION_HEADER
#define MOTION_HEADER 1

#include <stdint.h>

#include "dogbot/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif
  void MotionStep(void);

  void MotionResetCalibration(enum MotionHomedStateT defaultCalibrationState);

  void MotionUpdateIndex(bool state,float position,float velocity);

  bool MotionSetDemand(uint8_t mode,uint8_t timestamp,int16_t demand,int16_t torqueLimit);

  bool MotionReport(int16_t position,int16_t velocity,int16_t torque,enum PositionReferenceT posRef,uint8_t timeStamp);

  bool MotionOtherJointUpdate(int16_t position,int16_t torque,uint8_t mode,uint8_t timestamp);

  bool MotionCalZero(void);

  bool MotionSyncTime(void);

  void SetMotionUpdatePeriod(float period);

  extern enum MotionHomedStateT g_motionHomedState;
  extern enum PositionReferenceT g_motionPositionReference;
  extern enum ControlStateT g_controlState;
  extern float g_homeAngleOffset;      // Offset from phase position to actuator position.
  extern bool g_indicatorState;
  extern bool g_diagnosticMode;  //! When true extra parameter information is reported
  extern float g_absoluteMaxCurrent; // Maximum current allowed
  extern float g_actuatorRatio;
  extern float g_homeIndexPosition;

  extern int g_motionLastPositionTime;
  extern float g_motionUpdatePeriod;
  extern int g_motionPositionTime;
  extern int g_motionTimeOut;

  enum FaultCodeT LoadSetup(void);
  enum FaultCodeT SaveSetup(void);
  enum FaultCodeT RestoreFactorySetup(void);

#ifdef __cplusplus
}
#endif


#endif
