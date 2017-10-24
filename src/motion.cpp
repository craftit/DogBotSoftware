
#include "motion.h"
#include "pwm.h"
#include "mathfunc.h"
#include "coms.h"
#include "canbus.h"
#include "storedconf.h"

float g_angleOffset = 0;      // Offset from phase position to actuator position.
float g_motorPhase2RotationRatio = 7.0;
float g_actuatorRatio = g_motorPhase2RotationRatio * 21.0; // Gear ratio

float g_endStopMin = 0;    // Minimum angle
float g_endStopMax = M_PI*2; // Maximum angle
float g_absoluteMaxTorque = 20.0; // Maximum torque allowed
float g_uncalibratedTorqueLimit = 4.0; // Maximum torque allowed in un-calibrated mode.

const int g_positionIndexCount = 4;
float g_absoluteIndexPosition[g_positionIndexCount]; // Position of indexes
bool g_haveIndexPositionSample[g_positionIndexCount];
float g_measuredIndexPosition[g_positionIndexCount]; // Measured position

enum MotionCalibrationT g_motionCalibration = MC_Uncalibrated;
enum PositionReferenceT g_motionPositionReference = PR_Relative;
enum ControlStateT g_controlState = CS_StartUp;
enum FaultCodeT g_lastFaultCode = FC_Ok;
bool g_indicatorState = false;
bool g_newCalibrationData = false;

uint8_t g_otherJointId = 0;
float g_relativePositionGain = 1.0;
float g_relativePositionOffset = 0.0;


void MotionResetCalibration() {
  for(int i = 0;i < g_positionIndexCount;i++) {
    g_haveIndexPositionSample[i] = false;
  }
  bool changed = g_motionCalibration != MC_Uncalibrated;
  g_motionCalibration = MC_Uncalibrated;
  if(changed) {
    SendParamUpdate(CPI_PositionCal);
  }
}

static int EndStopUpdateOffset(bool newState,bool velocityPositive)
{
  return (newState ? 1 : 0) + ((velocityPositive ? 2 : 0));
}

void MotionUpdateEndStop(int num,bool state,float position,float velocity)
{
  if(g_motionCalibration != MC_Uncalibrated && num == 2) {

    int entry = EndStopUpdateOffset(state,velocity > 0);
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

  float meanS = 0;
  float meanC = 0;

  int off1,off2;
  for(int i = 0;i < 2;i++) {
    if(i == 0) {
      off1 = EndStopUpdateOffset(true,true);
      off2 = EndStopUpdateOffset(false,false);
    } else {
      off1 = EndStopUpdateOffset(false,true);
      off2 = EndStopUpdateOffset(true,false);
    }
    if(g_haveIndexPositionSample[off1] && g_haveIndexPositionSample[off2]) {

      float pos1 = g_measuredIndexPosition[off1] / g_actuatorRatio;
      float pos2 = g_measuredIndexPosition[off2] / g_actuatorRatio;
      meanS += sin(pos1);
      meanC += cos(pos1);
      meanS += sin(pos2);
      meanC += cos(pos2);
      points += 2;
    }
  }
  if(points <= 2)
    return false;
  value = atan2(meanS,meanC) * g_actuatorRatio;
  return true;
}

uint8_t g_requestedJointMode = 0;
int16_t g_requestedJointPosition = 0;
uint8_t g_otherJointMode = 0;
int16_t g_otherJointPosition = 0;
float g_otherPhaseOffset = 0; // Last reported position of other joint.

static float DemandToPhasePosition(int16_t position)
{
  return (((float) position) * g_actuatorRatio * M_PI * 4.0/ 65535.0);
}

static int16_t PhasePositionToDemand(float angle)
{
  return (angle * 65535.0) / (g_actuatorRatio * M_PI * 4.0);
}

bool UpdateRequestedPosition()
{
  float posf = DemandToPhasePosition(g_requestedJointPosition);

  if((g_requestedJointMode & 0x1) != 0) { // Calibrated position ?
    if(g_motionCalibration != MC_Calibrated)
      return false;
    posf += g_angleOffset;
  }

  if((g_motionPositionReference & 0x2) != 0) { // relative position ?
    // If we want a calibrated position and the other joint is uncalibrated then
    // give up.
    if((g_otherJointMode & 0x1) == 0 && (g_motionPositionReference & 0x1) != 0) {
      return false;
    }
    g_otherPhaseOffset =  DemandToPhasePosition(g_otherJointPosition) * g_relativePositionGain + g_relativePositionOffset;
    posf += g_otherPhaseOffset;
  }

  g_demandPhasePosition = posf;
  return true;
}


bool MotionSetPosition(uint8_t mode,int16_t position,uint16_t torqueLimit)
{
  g_requestedJointMode = mode;
  g_torqueLimit = ((float) torqueLimit) * g_absoluteMaxTorque / (65536.0);
  g_requestedJointPosition = position;

  return UpdateRequestedPosition();
}

bool MotionOtherJointUpdate(int16_t position,int16_t torque,uint8_t mode)
{
  g_otherJointPosition = position;
  g_otherJointMode = mode;
  if((g_motionPositionReference & 0x2) != 0)
    return UpdateRequestedPosition();
  return true;
}


bool MotionReport(int16_t position,int16_t torque,PositionReferenceT posRef)
{
  uint8_t mode = posRef & 0x3;

  // Report endstop switches.
#if 0
  if(palReadPad(GPIOC, GPIOC_PIN6))
    mode |= 1 << 3;
  if(palReadPad(GPIOC, GPIOC_PIN7))
    mode |= 1 << 4;
#endif
  if(palReadPad(GPIOC, GPIOC_PIN8))
    mode |= 1 << 3;

  if(g_canBridgeMode) {
    PacketServoReportC servo;
    servo.m_packetType = CPT_ServoReport;
    servo.m_deviceId = g_deviceId;
    servo.m_mode = mode;
    servo.m_position = position;
    servo.m_torque = torque;
    USBSendPacket(reinterpret_cast<uint8_t *>(&servo),sizeof(PacketServoReportC));
  }
  if(g_deviceId != 0) {
    CANSendServoReport(g_deviceId,position,torque,mode);
  }
  return true;
}

void CalibrationCheckFailed() {
  if(g_motionCalibration == MC_Uncalibrated)
    return ;
  g_motionCalibration = MC_Uncalibrated;
  if(g_motionPositionReference == PR_Absolute ||
      g_motionPositionReference == PR_OtherJointAbsolute) {
    FaultDetected(FC_PositionLost);
  }
  MotionResetCalibration();
  SendParamUpdate(CPI_PositionCal);
}

bool MotionCalZero()
{
  // Can only calibrate if pwm loop is running and we are updating our position.
  if(!g_pwmRun)
    return false;

  // Make sure we don't fly off somewhere.
  g_requestedJointMode = 0;
  g_requestedJointPosition = g_currentPhasePosition;

  g_angleOffset = g_currentPhasePosition;
  g_motionCalibration = MC_Calibrated;

  SendParamUpdate(CPI_CalibrationOffset);
  SendParamUpdate(CPI_PositionCal);
  return true;
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
        if(MotionEstimateOffset(g_angleOffset)) {
          g_motionCalibration = MC_Calibrated;
          SendParamUpdate(CPI_CalibrationOffset);
          SendParamUpdate(CPI_PositionCal);
        }
      }
      break;
    case MC_Calibrated: {
      // If we're calibrated, we're just going to check all is fine.
      if(g_newCalibrationData) {
        g_newCalibrationData = false;
        // If this is manual set, this will disagree and cause an error.
#if 0
        MotionEstimateOffset(g_angleOffset);
        SendParamUpdate(CPI_CalibrationOffset);
#else
#if 0
        float estimate = 0;
        if(MotionEstimateOffset(estimate)) {
          if(fabs(estimate - g_angleOffset) > M_PI/2.0) {
            CalibrationCheckFailed();
          }
        }
#endif
#endif
      }
    } break;
  }

  float position = g_currentPhasePosition;

  enum PositionReferenceT posRef = g_motionPositionReference;

  switch(g_motionPositionReference)
  {
    case PR_OtherJointAbsolute:
      position -= g_otherPhaseOffset;
      /* no break */
    case PR_Absolute: {
      if(g_motionCalibration == MC_Calibrated) {
        position -= g_angleOffset;
      } else {
        // Without calibration we can only send relative positions.
        position = g_currentPhasePosition;
        posRef = PR_Relative;
      }
      uint16_t reportPosition = PhasePositionToDemand(position);
      MotionReport(reportPosition,torque,posRef);
    } break;
    case PR_OtherJointRelative:
      position -= g_otherPhaseOffset;
      /* no break */
    case PR_Relative: {
      uint16_t reportPosition = PhasePositionToDemand(position);
      MotionReport(reportPosition,torque,posRef);
    } break;
  }
#endif
}


enum FaultCodeT LoadSetup(void) {
  if(!g_eeInitDone) {
    StoredConf_Init();
    g_eeInitDone = true;
  }
  StoredConf_Load(&g_storedConfig);

  g_deviceId = g_storedConfig.deviceId;
  g_otherJointId = g_storedConfig.otherJointId;
  g_relativePositionGain = g_storedConfig.m_relativePositionGain;
  g_relativePositionOffset = g_storedConfig.m_relativePositionOffset;
  g_motionPositionReference = (enum PositionReferenceT) g_storedConfig.m_motionPositionReference;

  // Setup angles.
  for(int i = 0;i < 12;i++) {
    g_phaseAngles[i][0] = g_storedConfig.phaseAngles[i][0];
    g_phaseAngles[i][1] = g_storedConfig.phaseAngles[i][1];
    g_phaseAngles[i][2] = g_storedConfig.phaseAngles[i][2];
  }

  // Should send an announce as this could change our id ?
  //uint8_t oldDevice = g_deviceId;
  //if(oldDevice != g_deviceId) {}

  return FC_Ok;
}

enum FaultCodeT SaveSetup(void) {
  if(!g_eeInitDone) {
    StoredConf_Init();
    g_eeInitDone = true;
  }

  g_storedConfig.configState = 1;
  g_storedConfig.deviceId = g_deviceId;
  g_storedConfig.otherJointId = g_otherJointId;

  g_storedConfig.m_relativePositionGain = g_relativePositionGain;
  g_storedConfig.m_relativePositionOffset =  g_relativePositionOffset;
  g_storedConfig.m_motionPositionReference = (int) g_motionPositionReference;


  for(int i = 0;i < 12;i++) {
    g_storedConfig.phaseAngles[i][0] = g_phaseAngles[i][0];
    g_storedConfig.phaseAngles[i][1] = g_phaseAngles[i][1];
    g_storedConfig.phaseAngles[i][2] = g_phaseAngles[i][2];
  }
  if(!StoredConf_Save(&g_storedConfig)) {
    return FC_InternalStoreFailed;
  }
  return FC_Ok;
}
