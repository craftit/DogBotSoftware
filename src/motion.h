#ifndef MOTION_HEADER
#define MOTION_HEADER 1

#include <stdint.h>

#include "dogbot/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif
  void MotionStep(void);

  void MotionResetCalibration(enum MotionHomedStateT defaultCalibrationState);

  void MotionUpdateIndex(int num,bool state,float position,float velocity);

  bool MotionSetPosition(uint8_t mode,int16_t position,uint16_t torqueLimit);

  bool MotionReport(int16_t position,int16_t torque,enum PositionReferenceT posRef);

  bool MotionOtherJointUpdate(int16_t position,int16_t torque,uint8_t mode);

  bool MotionCalZero(void);

  extern enum MotionHomedStateT g_motionHomedState;
  extern enum PositionReferenceT g_motionPositionReference;
  extern enum JointRelativeT g_motionJointRelative;
  extern enum ControlStateT g_controlState;
  extern float g_homeAngleOffset;      // Offset from phase position to actuator position.
  extern uint8_t g_otherJointId;  //! Id of other joint
  extern float g_relativePositionGain;
  extern float g_relativePositionOffset;
  extern bool g_indicatorState;
  extern float g_absoluteMaxCurrent; // Maximum current allowed
  extern float g_actuatorRatio;
  extern float g_homeIndexPosition;

  enum FaultCodeT LoadSetup(void);
  enum FaultCodeT SaveSetup(void);

#ifdef __cplusplus
}
#endif


#endif
