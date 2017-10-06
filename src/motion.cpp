
#include "motion.h"
#include "pwm.h"
#include "mathfunc.h"
#include "coms.h"
#include "canbus.h"

float g_angleOffset = 0;      // Offset from phase position to actuator position.
float g_actuatorRatio = 21.0; // Gear ratio

float g_endStopMin = 0;    // Minimum angle
float g_endStopMax = M_PI*2; // Maximum angle
float g_absoluteMaxTorque = 20.0; // Maximum torque allowed
float g_uncalibratedTorqueLimit = 4.0; // Maximum torque allowed in un-calibrated mode.

float g_absoluteIndexPosition[12]; // Position of indexes
bool g_haveIndexPositionSample[12];
float g_measuredIndexPosition[12]; // Measured position

MotionCalibrationT g_motionCalibration = MC_Uncalibrated;
PositionReferenceT g_motionPositionReference = PR_Relative;
enum ControlStateT g_controlState = CS_StartUp;
enum FaultCodeT g_lastFaultCode = FC_Ok;
bool g_indicatorState = false;
bool g_newCalibrationData = false;

void MotionResetCalibration() {
  for(int i = 0;i < 12;i++) {
    g_haveIndexPositionSample[i] = false;
  }
  g_motionCalibration = MC_Uncalibrated;
}

void MotionUpdateEndStop(int num,bool state,float position,float velocity)
{
  if(g_motionCalibration != MC_Uncalibrated) {
    int entry = num * 4 + (state ? 1 : 0) + ((velocity > 0 ? 2 : 0));
    if(!g_haveIndexPositionSample[entry]) {
      g_haveIndexPositionSample[entry] = true;
    }
    g_newCalibrationData = true;
    g_measuredIndexPosition[entry] = position;
  }
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

bool MotionSetPosition(uint8_t mode,uint16_t position,uint16_t torque)
{
  if((mode & 0x1) == 0) {
    PWMSetPosition(position,torque);
    return true;
  }
  if(g_motionCalibration != MC_Calibrated)
    return false;

  g_torqueLimit = ((float) torque) * g_absoluteMaxTorque / (2 * 65536.0);
  g_demandPhasePosition = (((float) position) * 7.0 * g_actuatorRatio * M_PI * 2.0/ 65535.0) + g_angleOffset;
  return true;
}

bool MotionReport(uint16_t position,int16_t torque,PositionReferenceT posRef)
{
  uint8_t mode = posRef & 0x3;

  // Report endstop switches.
  if(palReadPad(GPIOC, GPIOC_PIN6))
    mode |= 1 << 3;
  if(palReadPad(GPIOC, GPIOC_PIN7))
    mode |= 1 << 4;
  if(palReadPad(GPIOC, GPIOC_PIN8))
    mode |= 1 << 5;

  if(g_canBridgeMode) {
    PacketServoReportC servo;
    servo.m_packetType = CPT_ServoReport;
    servo.m_deviceId = g_deviceId;
    servo.m_mode = mode;
    servo.m_position = position;
    servo.m_torque = torque;
    SendPacket(reinterpret_cast<uint8_t *>(&servo),sizeof(PacketServoReportC));
  }
  if(g_deviceId != 0) {
    CANSendServoReport(g_deviceId,position,torque,mode);
  }
  return true;
}

void CalibrationCheckFailed() {
  g_motionCalibration = MC_Uncalibrated;
  g_torqueLimit = 0;
  g_controlMode = CM_Fault;
}

void MotionStep()
{
#if 1
  int16_t torque = g_torqueAverage * (65535.0/g_absoluteMaxTorque);

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
    case MC_Calibrated: {
      // If we're calibrated, we're just going to check all is fine.
      if(g_newCalibrationData) {
        float estimate;
        g_newCalibrationData = false;
        if(!MotionEstimateOffset(estimate)) {
          CalibrationCheckFailed();
        }
        if(fabs(estimate - g_angleOffset) > (M_PI/2.0)) {
          CalibrationCheckFailed();
        }
      }
    } break;
  }

  // Check we've got a calibrated position
  if(g_motionPositionReference == PR_Absolute && g_motionCalibration != MC_Calibrated)
    g_motionPositionReference = PR_Relative;

  float position = g_currentPhasePosition;

  enum PositionReferenceT posRef = g_motionPositionReference;

  switch(g_motionPositionReference)
  {
    case PR_OtherJointRelative:
    case PR_OtherJointAbsolute:
      // TODO: Correct for other joint offset.
    case PR_Absolute:
      if(g_motionCalibration == MC_Calibrated) {
        position -= g_angleOffset;
      } else {
        // Without calibration we can only send relative positions.
        position = g_currentPhasePosition;
        posRef = PR_Relative;
      }
    /* no break */
    case PR_Relative: {
      uint16_t reportPosition = (position * 65535.0) / (7.0 * g_actuatorRatio * M_PI * 2.0) ;
      MotionReport(reportPosition,torque,posRef);
    } break;
  }
#endif
}
