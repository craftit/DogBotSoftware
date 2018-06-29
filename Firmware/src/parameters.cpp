
#include <stdint.h>

#include "hal.h"
#include "coms.h"
#include "canbus.h"
#include "dogbot/protocol.h"
#include "chmboxes.h"
#include "pwm.h"
#include "mathfunc.h"
#include "motion.h"
#include "drv8503.h"
#include "hal_channels.h"
#include <string.h>

uint8_t g_debugIndex = 0x55;

void EnableFanPower(bool enable)
{
  if(enable)
    palSetPad(GPIOA, GPIOA_PIN7); // Turn off aux power
  else
    palClearPad(GPIOA, GPIOA_PIN7); // Turn on aux power
}

bool HasSensorPower()
{
  return palReadPad(GPIOB, GPIOB_PIN12) != 0;
}

void EnableSensorPower(bool enable)
{
  if(enable)
    palSetPad(GPIOB, GPIOB_PIN12); // Turn off sensor power
  else
    palClearPad(GPIOB, GPIOB_PIN12); // Turn off sensor power
}


void SetMotorControlMode(PWMControlDynamicT controlMode)
{
  g_controlMode = controlMode;
  SendParamUpdate(CPI_PWMMode);
}


bool SetParam(enum ComsParameterIndexT index,union BufferTypeT *data,int len)
{
  switch(index )
  {
    case CPI_FirmwareVersion:
      return false; // Can't set this
    case CPI_PWMState:
      if(len < 1)
        return false;
      if(data->uint8[0] > 0) {
        PWMRun();
      } else {
        PWMStop();
      }
      break;
    case CPI_PWMMode:
      if(len != 1)
        return false;
      if(data->uint8[0] >= (int) CM_Final)
        return false;
      // Don't let the user change this when in an
      // emergency or shutdown mode.
      switch(g_controlState)
      {
        case CS_EmergencyStop:
        case CS_SafeStop:
          g_controlMode = CM_Brake;
          break;
        default:
          g_controlMode = (PWMControlDynamicT) data->uint8[0];
          break;
      }
      break;
    case CPI_PWMFullReport:
      if(len != 1)
        return false;
      g_pwmFullReport = data->uint8[0] > 0;
      break;
    case CPI_CANBridgeMode:
      if(len != 1)
        return false;
      g_canBridgeMode = data->uint8[0] > 0;
      break;
    case CPI_IndexSensor:
    case CPI_BoardUID:
    case CPI_DRV8305_01:
    case CPI_DRV8305_02:
    case CPI_DRV8305_03:
    case CPI_DRV8305_04:
    case CPI_DRV8305_05:
    case CPI_VSUPPLY:
    case CPI_5VRail:
    case CPI_DriveTemp:
    case CPI_MotorTemp:
    case CPI_MotorResistance:
    case CPI_MotorInductance:
    case CPI_PhaseVelocity:
    case CPI_HallSensors:
    case CPI_MotorOffsetVoltage:
    case CPI_DeviceType:
    case CPI_TIM1_SR:
      break;

    case CPI_CalibrationOffset:
      if(len != 4)
        return false;
      switch(g_motionHomedState)
      {
        case MHS_Lost:
          MotionResetCalibration(MHS_SoftHomed);
        case MHS_Measuring:
        case MHS_SoftHomed:
          g_homeAngleOffset = data->float32[0] * g_actuatorRatio;
          g_motionHomedState = MHS_SoftHomed;
          SendParamUpdate(CPI_HomedState);
          break;
        case MHS_Homed:
          SendError(CET_UnavailableInCurrentMode,CPT_SetParam,index);
          break;
      }
      return true;

    case CPI_HomedState: {
      if(len != 1)
        return false;
      enum MotionHomedStateT newCal = (enum MotionHomedStateT) data->uint8[0];
      switch(newCal)
      {
        case MHS_Lost:
          MotionResetCalibration(MHS_Lost);
          break;
        case MHS_Measuring:
          MotionResetCalibration(MHS_Measuring);
          break;
        case MHS_SoftHomed: {
          enum MotionHomedStateT oldState = g_motionHomedState;
          g_motionHomedState = MHS_SoftHomed;
          if(oldState != g_motionHomedState)
            SendParamUpdate(CPI_HomedState);
        } break;
        case MHS_Homed:
          // Let user know it hasn't changed.
          if(g_motionHomedState  != newCal)
            SendParamUpdate(CPI_HomedState);
          return false;
      }
    } break;
    case CPI_PositionRef: {
      if(len != 1)
        return false;
      enum PositionReferenceT posRef = (enum PositionReferenceT) data->uint8[0];
      switch(posRef)
      {
        case PR_Relative:
        case PR_Absolute:
          g_motionPositionReference = posRef;
          break;
        default:
          return false;
      }
    } break;
    case CPI_ControlState: {
      if(len != 1)
        return false;
      enum ControlStateT newState = (enum ControlStateT) data->uint8[0];
      if(!ChangeControlState(newState,SCS_UserRequest))
        return false;
    } break;
    case CPI_FaultCode:
      // Just clear it.
      g_lastFaultCode = FC_Ok;
      break;
    case CPI_FaultState:
      // Just clear it.
      g_faultState = 0;
      break;
    case CPI_Indicator:
      if(len != 1)
        return false;
      g_indicatorState = data->uint8[0] > 0;
      break;
    case CPI_OtherJoint: // Obsolete.
      return false;
      break;
    case CPI_OtherJointGain:  // Obsolete.
      return false;
    case CPI_OtherJointOffset:  // Obsolete.
      return false;
    case CPI_DebugIndex:
      g_debugIndex = len;
      break;
    case CPI_MotorIGain:
      if(len != 4)
        return false;
      g_motor_i_gain = data->float32[0];
      break;
    case CPI_MotorPGain:
      if(len != 4)
        return false;
      g_motor_p_gain = data->float32[0];
      break;
    case CPI_VelocityPGain:
      if(len != 4)
        return false;
      g_velocityPGain =  data->float32[0];
      break;
    case CPI_VelocityIGain:
      if(len != 4)
        return false;
      g_velocityIGain =  data->float32[0];
      break;
    case CPI_DemandPhaseVelocity:
      if(len != 4)
        return false;
      g_demandPhaseVelocity = data->float32[0];
      break;
    case CPI_VelocityLimit:
      if(len != 4)
        return false;
      g_velocityLimit = data->float32[0];
      break;
    case CPI_PositionGain:
      if(len != 4)
        return false;
      g_positionGain = data->float32[0];
      break;
    case CPI_MaxCurrent:
      if(len != 4)
        return false;
      g_absoluteMaxCurrent = data->float32[0];
      break;
    case CPI_homeIndexPosition:
      if(len != 4)
        return false;
      g_homeIndexPosition = data->float32[0];
      break;
    case CPI_MinSupplyVoltage:
      if(len != 4)
        return false;
      g_minSupplyVoltage = data->float32[0];
      break;
    //case CPI_ANGLE_CAL: // 12 Values
    case CPI_ANGLE_CAL_0:
    case CPI_ANGLE_CAL_1:
    case CPI_ANGLE_CAL_2:
    case CPI_ANGLE_CAL_3:
    case CPI_ANGLE_CAL_4:
    case CPI_ANGLE_CAL_5:
    case CPI_ANGLE_CAL_6:
    case CPI_ANGLE_CAL_7:
    case CPI_ANGLE_CAL_8:
    case CPI_ANGLE_CAL_9:
    case CPI_ANGLE_CAL_10:
    case CPI_ANGLE_CAL_11:
    case CPI_ANGLE_CAL_12:
    case CPI_ANGLE_CAL_13:
    case CPI_ANGLE_CAL_14:
    case CPI_ANGLE_CAL_15:
    case CPI_ANGLE_CAL_16:
    case CPI_ANGLE_CAL_17:
    {
      int reg = ((int) index - CPI_ANGLE_CAL);
      if(len != 6) return false;
      for(int i = 0;i < 3;i++)
        g_phaseAngles[reg][i] = data->uint16[i];
    } break;

    case CPI_USBPacketDrops:
      g_usbDropCount = 0;
      break;
    case CPI_USBPacketErrors:
      g_usbErrorCount = 0;
      break;
    case CPI_CANPacketDrops:
      g_canDropCount = 0;
      break;
    case CPI_CANPacketErrors:
      g_canErrorCount = 0;
      break;
    case CPI_MainLoopTimeout:
      g_mainLoopTimeoutCount = 0;
      break;
    case CPI_JointRelative: // Obsolete
      return false;
    case CPI_FanMode: {
      if(len != 1)
        return false;
      enum FanModeT fanMode = (enum FanModeT) data->uint8[0];
      switch(fanMode)
      {
        case FM_Off:
          EnableFanPower(false);
          break;
        case FM_On:
          EnableFanPower(true);
          break;
        case FM_Auto:
          break;
        default:
          return false;
      }
      g_fanMode = fanMode;
    } break;
    case CPI_FanTemperatureThreshold: {
      if(len != 4)
        return false;
      g_fanTemperatureThreshold = data->float32[0];
    } break;
    case CPI_FanState: {
      return false;
    } break;

    case CPI_JointRole: {
      if(len != 1)
        return false;
      g_jointRole = (enum JointRoleT) data->uint8[0];
      break;
    }
    case CPI_SafetyMode: {
      if(len != 1)
        return false;
      SafetyModeT sm = (enum SafetyModeT) data->uint8[0];
      switch(g_controlState) {
        default:
          SendParamUpdate(index);
          return false;
        case CS_Standby:
          switch(sm) {
            case SM_GlobalEmergencyStop:
            case SM_MasterEmergencyStop:
            case SM_LocalStop:
            case SM_Unknown:
              g_safetyMode = sm;
              break;
            default:
              // If unrecognised, fall back to unknown
              g_safetyMode = SM_Unknown;
              SendParamUpdate(index);
              return false;
          }
        break;
      }
    } break;

    case CPI_EndStopEnable: {
      if(len != 1)
        return false;
      bool newValue = data->uint8[0] != 0;
      if(newValue)
        SetupEndStops();
      g_endStopEnable = newValue;
    } break;
    case CPI_EndStopStart: {
      if(len != 4)
        return false;
      g_endStopMin = data->float32[0];
      SetupEndStops();
    } break;
    case CPI_EndStopStartBounce:{
      if(len != 4)
        return false;
      g_endStopStartBounce = data->float32[0];
      SetupEndStops();
    } break;

    case CPI_EndStopFinal:{
      if(len != 4)
        return false;
      g_endStopMax = data->float32[0];
      SetupEndStops();
    } break;
    case CPI_EndStopEndBounce:{
      if(len != 4)
        return false;
      g_endStopEndBounce = data->float32[0];
      SetupEndStops();
    } break;
    case CPI_EndStopTargetBreakForce:{
      if(len != 4)
        return false;
      g_endStopTargetBreakCurrent = data->float32[0];
      SetupEndStops();
    } break;
    case CPI_EndStopLimitBreakForce: {
      if(len != 4)
        return false;
      g_endStopMaxBreakCurrent = data->float32[0];
      SetupEndStops();
    } break;
    case CPI_JointInertia:{
      if(len != 4)
        return false;
      g_jointInertia = data->float32[0];
      SetupEndStops();
    } break;
    case CPI_EndStopPhaseAngles:
      return false;
    case CPI_ServoReportFrequency:
      if(len != 4)
        return false;
      SetServoReportRate(data->float32[0]);
      return true;
    case CPI_PWMFrequency:
      // Setting the PWM frequency is not supported at the moment.
      return false;
    case CPI_MotionUpdatePeriod: {
      if(len != 2)
        return false;
      g_motionUpdatePeriod = data->int16[0];
    } return false;
    case CPI_SupplyVoltageScale: {
      if(len != 4)
        return false;
      float newValue = data->float32[0];
      // Limit range to somewhat sensible values.
      if(newValue < 0.8 || newValue > 1.4)
        return false;
      g_supplyVoltageScale = newValue;
    } return true;
    case CPI_FINAL:
      return false;
    default:
      return false;
  }

  SendParamUpdate(index);

  return true;
}


// Up to 8 bytes of data may be returned by this function.

bool ReadParam(enum ComsParameterIndexT index,int *len,union BufferTypeT *data)
{
  switch(index)
  {
    case CPI_DeviceType:
      *len = 1;
      data->uint8[0] = (int) DT_MotorDriver;
      break;
    case CPI_FirmwareVersion:
      *len = 1;
      data->uint8[0] = DOGBOT_FIRMWARE_VERSION;
      break;
    case CPI_PWMState:
      *len = 1;
      data->uint8[0] = g_pwmThreadRunning;
      break;
    case CPI_PWMMode:
      *len = 1;
      data->uint8[0] = g_controlMode;
      break;
    case CPI_PWMFullReport:
      *len = 1;
      data->uint8[0] = g_pwmFullReport;
      break;
    case CPI_CANBridgeMode:
      *len = 1;
      data->uint8[0] = g_canBridgeMode;
      break;
    case CPI_BoardUID:
      *len = 8;
      data->uint32[0] = g_nodeUId[0];
      data->uint32[1] = g_nodeUId[1];
      break;
    case CPI_DRV8305_01:
    case CPI_DRV8305_02:
    case CPI_DRV8305_03:
    case CPI_DRV8305_04:
    case CPI_DRV8305_05: {
      int reg = ((int) index - CPI_DRV8305)+1;
      *len = 2;
      data->uint16[0] = Drv8503ReadRegister(reg);
    } break;

    case CPI_TIM1_SR: {
      stm32_tim_t *tim = (stm32_tim_t *)TIM1_BASE;
      *len = 2;
      data->uint16[0] = (uint16_t) tim->SR;
    } break;
    case CPI_VSUPPLY: {
      unsigned val = g_vbus_voltage * 1000.0f;
      *len = 2;
      data->uint16[0] = val;
    } break;
    case CPI_5VRail: {
      *len = 4;
      data->float32[0] = Read5VRailVoltage();
    } break;
    case CPI_HomedState:
      *len = 1;
      data->uint8[0] = (int) g_motionHomedState;
      break;
    case CPI_PositionRef:
      *len = 1;
      data->uint8[0] = (int) g_motionPositionReference;
      break;
    case CPI_ControlState:
      *len = 1;
      data->uint8[0] = (int) g_controlState;
      break;
    case CPI_FaultCode:
      *len = 1;
      data->uint8[0] = (int) g_lastFaultCode;
      break;
    case CPI_Indicator:
      *len = 1;
      data->uint8[0] = (int) g_indicatorState;
      break;
    case CPI_CalibrationOffset:
      *len = 4;
      data->float32[0] = g_homeAngleOffset / g_actuatorRatio;
      break;
    case CPI_DriveTemp:
      *len = 4;
      data->float32[0] = g_driveTemperature;
      break;
    case CPI_MotorTemp:
      *len = 4;
      data->float32[0] = g_motorTemperature;
      break;
    case CPI_OtherJoint: // Obsolete.
      *len = 1;
      data->uint8[0] = 0;
      break;
    case CPI_OtherJointGain: // Obsolete
      *len = 4;
      data->float32[0] = 0;
      break;
    case CPI_OtherJointOffset: // Obsolete
      *len = 4;
      data->float32[0] = 0;
      break;
    case CPI_PhaseVelocity:
      *len = 4;
      data->float32[0] = g_currentPhaseVelocity;
      break;
    case CPI_PositionGain:
      *len = 4;
      data->float32[0] = g_positionGain;
      break;
    //case CPI_ANGLE_CAL: // 12 Values
    case CPI_ANGLE_CAL_0:
    case CPI_ANGLE_CAL_1:
    case CPI_ANGLE_CAL_2:
    case CPI_ANGLE_CAL_3:
    case CPI_ANGLE_CAL_4:
    case CPI_ANGLE_CAL_5:
    case CPI_ANGLE_CAL_6:
    case CPI_ANGLE_CAL_7:
    case CPI_ANGLE_CAL_8:
    case CPI_ANGLE_CAL_9:
    case CPI_ANGLE_CAL_10:
    case CPI_ANGLE_CAL_11:
    case CPI_ANGLE_CAL_12:
    case CPI_ANGLE_CAL_13:
    case CPI_ANGLE_CAL_14:
    case CPI_ANGLE_CAL_15:
    case CPI_ANGLE_CAL_16:
    case CPI_ANGLE_CAL_17:
    {
      int reg = ((int) index - CPI_ANGLE_CAL);
      *len = 6;
      for(int i = 0;i < 3;i++)
        data->uint16[i] = g_phaseAngles[reg][i];
    } break;
    case CPI_DebugIndex:
      *len = 1;
      data->uint8[0] = g_debugIndex;
      break;
    case CPI_MotorResistance:
      *len = 4;
      data->float32[0] = g_phaseResistance;
      break;
    case CPI_MotorInductance:
      *len = 4;
      data->float32[0] = g_phaseInductance;
      break;
    case CPI_MotorIGain:
      *len = 4;
      data->float32[0] = g_motor_i_gain;
      break;
    case CPI_MotorPGain:
      *len = 4;
      data->float32[0] = g_motor_p_gain;
      break;
    case CPI_VelocityPGain:
      *len = 4;
      data->float32[0] = g_velocityPGain;
      break;
    case CPI_VelocityIGain:
      *len = 4;
      data->float32[0] = g_velocityIGain;
      break;
    case CPI_DemandPhaseVelocity:
      *len = 4;
      data->float32[0] = g_demandPhaseVelocity;
      break;
    case CPI_VelocityLimit:
      *len = 4;
      data->float32[0] = g_velocityLimit;
      break;
    case CPI_MaxCurrent:
      *len = 4;
      data->float32[0] = g_absoluteMaxCurrent;
      break;
    case CPI_homeIndexPosition:
      *len = 4;
      data->float32[0] = g_homeIndexPosition;
      break;
    case CPI_HallSensors:
      *len = 6;
      data->uint16[0] = g_hall[0];
      data->uint16[1] = g_hall[1];
      data->uint16[2] = g_hall[2];
      break;
    case CPI_MinSupplyVoltage:
      *len = 4;
      data->float32[0] = g_minSupplyVoltage;
      break;
    case CPI_USBPacketDrops:
      *len = 4;
      data->uint32[0] = g_usbDropCount;
      break;
    case CPI_USBPacketErrors:
      *len = 4;
      data->uint32[0] = g_usbErrorCount;
      break;
    case CPI_CANPacketDrops:
      *len = 4;
      data->uint32[0] = g_canDropCount;
      break;
    case CPI_CANPacketErrors:
      *len = 4;
      data->uint32[0] = g_canErrorCount;
      break;
    case CPI_FaultState:
      *len = 4;
      data->uint32[0] = g_faultState;
      break;
    case CPI_IndexSensor:
      *len = 1;
      data->uint8[0] = palReadPad(GPIOC, GPIOC_PIN8);
      break;
    case CPI_MainLoopTimeout:
      *len = 4;
      data->uint32[0] = g_mainLoopTimeoutCount;
      break;
    case CPI_JointRelative: // Obsolete
      *len = 1;
      data->uint8[0] = 0;
      break;
    case CPI_FanMode: {
      *len = 1;
      data->uint8[0] = g_fanMode;
    } break;
    case CPI_FanTemperatureThreshold: {
      *len = 4;
      data->float32[0] = g_fanTemperatureThreshold;
    } break;
    case CPI_FanState: {
      *len = 1;
      int i = palReadPad(GPIOA, GPIOA_PIN7); // Pin State.
      if(palReadPad(GPIOB, GPIOB_PIN11)) // Status feedback
        i |= 2;
      data->uint8[0] = i;
    } break;

    case CPI_JointRole: {
      *len = 1;
      data->uint8[0] = g_jointRole;
      break;
    }
    case CPI_SafetyMode: {
      *len = 1;
      data->uint8[0] = g_safetyMode;
    } break;

    case CPI_EndStopEnable: {
      *len = 1;
      data->uint8[0] = g_endStopEnable;
    } break;
    case CPI_EndStopStart: {
      *len = 4;
      data->float32[0] = g_endStopMin;
    } break;
    case CPI_EndStopStartBounce:{
      *len = 4;
      data->float32[0] = g_endStopStartBounce;
    } break;
    case CPI_EndStopFinal:{
      *len = 4;
      data->float32[0] = g_endStopMax;
    } break;
    case CPI_EndStopEndBounce:{
      *len = 4;
      data->float32[0] = g_endStopEndBounce;
    } break;
    case CPI_EndStopTargetBreakForce:{
      *len = 4;
      data->float32[0] = g_endStopTargetBreakCurrent;
    } break;
    case CPI_EndStopLimitBreakForce: {
      *len = 4;
      data->float32[0] = g_endStopMaxBreakCurrent;
    } break;
    case CPI_JointInertia:{
      *len = 4;
      data->float32[0] = g_jointInertia;
    } break;
    case CPI_MotorOffsetVoltage:{
      *len = 4;
      data->float32[0] = g_phaseOffsetVoltage;
    } break;
    case CPI_EndStopPhaseAngles: {
      *len = 8;
      data->float32[0] = g_endStopPhaseMin;
      data->float32[1] = g_endStopPhaseMax;
    } break;
    case CPI_ServoReportFrequency: {
      *len = 4;
      data->float32[0] = GetServoReportRate();
    } break;
    case CPI_PWMFrequency: {
      *len = 4;
      data->float32[0] = g_PWMFrequency;
    } break;
    case CPI_MotionUpdatePeriod: {
      *len = 2;
      data->int16[0] = g_motionUpdatePeriod;
    } break;
    case CPI_SupplyVoltageScale: {
      *len = 4;
      data->float32[0] = g_supplyVoltageScale;
    } break;
    case CPI_FINAL:
    default: return false;
  }
  return true;
}
