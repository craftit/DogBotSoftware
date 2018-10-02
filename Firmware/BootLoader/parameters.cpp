
#include "hal.h"
#include "bmc.h"
#include "canbus.h"
#include "bootloader.h"

#if 0
bool g_canBridgeMode = false;
void SendParamUpdate(enum ComsParameterIndexT paramIndex) {
#if 0
  if(g_deviceId == 0 || g_canBridgeMode) {
    if(!USBReadParamAndReply(paramIndex)) {
      USBSendError(g_deviceId,CET_ParameterOutOfRange,CPT_ReadParam,(uint8_t) paramIndex);
    }
  }
  if(g_deviceId != 0) {
    CANSendParam(paramIndex);
#if 0
    if(!CANSendReadParam(g_deviceId,paramIndex)) {
      USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_ReadParam,(uint8_t) paramIndex);
    }
#endif
  }
#else
  if(g_deviceId != 0) {
    CANSendParam(paramIndex);
  }
#endif
}
#endif


uint8_t g_debugIndex = 0x55;

bool SetParam(enum ComsParameterIndexT index,union BufferTypeT *dataBuff,int len)
{
  switch(index)
  {
    case CPI_CANBridgeMode:
      if(len != 1) return false;
      g_canBridgeMode = dataBuff->uint8[0] > 0;
      break;
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
      g_indicatorState = dataBuff->uint8[0] > 0;
      break;
    case CPI_DebugIndex:
      g_debugIndex = len;
      break;
    case CPI_USBPacketDrops:
      //g_usbDropCount = 0;
      break;
    case CPI_USBPacketErrors:
      //g_usbErrorCount = 0;
      break;
    case CPI_CANPacketDrops:
      g_canDropCount = 0;
      break;
    case CPI_CANPacketErrors:
      g_canErrorCount = 0;
      break;
    case CPI_ControlState: {
      if(len != 1)
        return false;
      enum ControlStateT newState = (enum ControlStateT) dataBuff->uint8[0];
      if(newState == CS_Standby ||
          newState == CS_BootLoader)
        g_controlState = newState;
    } break;
    case CPI_MainLoopTimeout:
    case CPI_FirmwareVersion:
    case CPI_PWMState:
    case CPI_PWMMode:
    case CPI_PWMFullReport:
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
    case CPI_CalibrationOffset:
    case CPI_HomedState:
    case CPI_PositionRef:
    case CPI_OtherJoint:
    case CPI_OtherJointGain:
    case CPI_OtherJointOffset:
    case CPI_MotorIGain:
    case CPI_MotorPGain:
    case CPI_VelocityPGain:
    case CPI_VelocityIGain:
    case CPI_DemandPhaseVelocity:
    case CPI_VelocityLimit:
    case CPI_PositionGain:
    case CPI_MaxCurrent:
    case CPI_homeIndexPosition:
    case CPI_MinSupplyVoltage:
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
    case CPI_JointRelative:
    case CPI_FanMode:
    case CPI_FanTemperatureThreshold:
    case CPI_FanState:
    case CPI_SafetyMode:
    case CPI_JointRole:
    case CPI_EndStopEnable:
    case CPI_EndStopStart:
    case CPI_EndStopStartBounce:
    case CPI_EndStopFinal:
    case CPI_EndStopEndBounce:
    case CPI_EndStopTargetBreakForce:
    case CPI_EndStopLimitBreakForce:
    case CPI_JointInertia:
    case CPI_EndStopPhaseAngles:
    case CPI_ServoReportFrequency:
    case CPI_PWMFrequency:
    case CPI_MotionUpdatePeriod:
    case CPI_SupplyVoltageScale:
    case CPI_CurrentLimit:
    case CPI_RequestedPlatformActivity:
    case CPI_PlatformActivity:
    case CPI_FINAL:
      return false;
//    default:
//      return false;
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
      data->uint8[0] = (int) DT_BootLoader;
      break;
    case CPI_FirmwareVersion:
      *len = 1;
      data->uint8[0] = DOGBOT_FIRMWARE_VERSION;
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
    case CPI_VSUPPLY: {
      //unsigned val = g_vbus_voltage * 1000.0f;
      *len = 2;
      data->uint16[0] = 0;//val;
    } break;
    case CPI_5VRail: {
      *len = 4;
      data->float32[0] = 0;//Read5VRailVoltage();
    } break;
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
    case CPI_DebugIndex:
      *len = 1;
      data->uint8[0] = g_debugIndex;
      break;
    case CPI_MinSupplyVoltage:
      *len = 4;
      data->float32[0] = 0;//g_minSupplyVoltage;
      break;
    case CPI_USBPacketDrops:
      *len = 4;
      data->uint32[0] = 0;//g_usbDropCount;
      break;
    case CPI_USBPacketErrors:
      *len = 4;
      data->uint32[0] = 0;//g_usbErrorCount;
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
    case CPI_MainLoopTimeout:
    case CPI_PWMState:
    case CPI_PWMMode:
    case CPI_PWMFullReport:
    case CPI_DRV8305_01:
    case CPI_DRV8305_02:
    case CPI_DRV8305_03:
    case CPI_DRV8305_04:
    case CPI_DRV8305_05:
    case CPI_TIM1_SR:
    case CPI_HomedState:
    case CPI_PositionRef:
    case CPI_CalibrationOffset:
    case CPI_DriveTemp:
    case CPI_MotorTemp:
    case CPI_OtherJoint:
    case CPI_OtherJointGain:
    case CPI_OtherJointOffset:
    case CPI_PhaseVelocity:
    case CPI_PositionGain:
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
    case CPI_MotorResistance:
    case CPI_MotorInductance:
    case CPI_MotorIGain:
    case CPI_MotorPGain:
    case CPI_VelocityPGain:
    case CPI_VelocityIGain:
    case CPI_DemandPhaseVelocity:
    case CPI_VelocityLimit:
    case CPI_MaxCurrent:
    case CPI_homeIndexPosition:
    case CPI_HallSensors:
    case CPI_IndexSensor:
    case CPI_JointRelative:
    case CPI_FanMode:
    case CPI_FanTemperatureThreshold:
    case CPI_FanState:
    case CPI_ServoReportFrequency:
    case CPI_PWMFrequency:
    default:
      return false;
  }
  return true;
}

