#ifndef MOTION_HEADER
#define MOTION_HEADER 1

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
  void MotionStep(void);

  void MotionResetCalibration(void);

  void MotionUpdateEndStop(int num,bool state,float position,float velocity);

  bool MotionSetPosition(uint16_t position,uint16_t torque);

  bool MotionReport(uint16_t position,uint16_t torque);

  enum MotionCalibrationT {
    MC_Uncalibrated,
    MC_Measuring,
    MC_Calibrated,
    MC_Update,
    MC_CheckError
  };

  enum MotionStateT {
    MS_Stopped,
    MS_Relative,
    MS_Absolute,
    MS_Joint
  };

  extern enum MotionCalibrationT g_motionCalibration;
  extern enum MotionStateT g_motionState;



#ifdef __cplusplus
}
#endif


#endif
