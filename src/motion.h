#ifndef MOTION_HEADER
#define MOTION_HEADER 1

#ifdef __cplusplus
extern "C" {
#endif
  void MotionStep(void);

  void MotionResetCalibration(void);

  void MotionUpdateEndStop(int num,bool state,float position,float velocity);

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

  extern MotionCalibrationT g_motionCalibration;
  extern MotionStateT g_motionState;


#ifdef __cplusplus
}
#endif


#endif
