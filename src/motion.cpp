
#include "motion.h"
#include "pwm.h"
#include "mathfunc.h"

float g_angleOffset = 0;      // Offset from phase position to actuator position.
float g_actuatorRatio = 21.0; // Gear ratio

float g_endStopMin = 0;    // Minimum angle
float g_endStopMax = 3.14; // Maximum angle

float g_uncalibratedTorqueLimit = 4.0; // Maximum torque allowed in un-calibrated mode.

float g_absoluteIndexPosition[12]; // Position of indexes
bool g_haveIndexPositionSample[12];
float g_measuredIndexPosition[12]; // Measured position

MotionCalibrationT g_motionCalibration = MC_Uncalibrated;
MotionStateT g_motionState = MS_Stopped;

bool g_newCalibrationData = false;

void MotionResetCalibration() {
  for(int i = 0;i < 12;i++) {
    g_haveIndexPositionSample[i] = false;
  }
  g_motionCalibration = MC_Uncalibrated;
}

void MotionUpdateEndStop(int num,bool state,float position,float velocity)
{
  int entry = num * 4 + (state ? 1 : 0) + ((velocity > 0 ? 2 : 0));
  if(!g_haveIndexPositionSample[entry]) {
    g_haveIndexPositionSample[entry] = true;
  }
  g_newCalibrationData = true;
  g_measuredIndexPosition[entry] = position;

}

bool MotionEstimateOffset(float &value)
{
  int points = 0;
  float offset = 0;
  for(int i = 0;i < 12;i++) {
    if(!g_haveIndexPositionSample[i])
      continue;
    offset += g_measuredIndexPosition[i] - g_absoluteIndexPosition[i];
    points++;
  }
  if(points <= 0)
    return false;
  value = offset / (float) points;
  return true;

}

bool MotionSetPosition(uint16_t position,uint16_t torque)
{
  if(g_motionCalibration != MC_Calibrated)
    return false;

  g_torqueLimit = ((float) torque) * 10.0 / (65535.0);
  g_demandPhasePosition = (((float) position) * 7.0 * g_actuatorRatio * M_PI * 2.0/ 65535.0) + g_angleOffset;
  return true;
}


void MotionStep()
{
  // Should we
  switch(g_motionCalibration)
  {
    case MC_Uncalibrated:
      break;
    case MC_Measuring:
      // Establishing a new calibration
      if(g_newCalibrationData) {
        g_newCalibrationData = false;
        if(MotionEstimateOffset(g_angleOffset))
          g_motionCalibration = MC_Calibrated;
      }
      break;
    case MC_Update: {
      // ...
    } break;
    case MC_Calibrated: {
      // If we're calibrated, we're just going to check all is fine.
      if(g_newCalibrationData) {
        float estimate;
        g_newCalibrationData = false;
        if(!MotionEstimateOffset(estimate)) {
          g_motionCalibration = MC_CheckError;
          g_motionState = MS_Stopped;
        }
        if(fabs(estimate - g_angleOffset) > (M_PI/2.0)) {
          g_motionCalibration = MC_CheckError;
          g_motionState = MS_Stopped;
        }
      }
    } break;
    case MC_CheckError: {
      // Error state
      g_motionState = MS_Stopped;
    } break;

  }

  // Check we've got a calibrated position
  if(g_motionState == MS_Absolute && g_motionCalibration != MC_Calibrated)
    g_motionState = MS_Relative;

  switch(g_motionState)
  {
    case MS_Stopped:
      g_torqueLimit = 0;
      g_controlMode = CM_Idle;
      break;
    case MS_Relative:

      break;
    case MS_Absolute:
      break;
    case MS_Joint:
      break;
  }

}
