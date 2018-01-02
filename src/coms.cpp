
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


bool g_canBridgeMode = false;
bool g_comsInitDone = false;

int g_usbDropCount = 0;
int g_usbErrorCount = 0;

bool USBSendSync(void)
{
  uint8_t buff[2];
  int at = 0;
  buff[at++] = CPT_Sync; // Type
  return USBSendPacket(buff,at);
}

bool USBSendPing(uint8_t deviceId)
{
  PacketPingPongC pkt;
  pkt.m_packetType = CPT_Ping;
  pkt.m_deviceId = deviceId;
  return USBSendPacket((uint8_t *) &pkt,sizeof(struct PacketPingPongC));
}

// Error codes
//  1 - Unexpected packet.
//  2 - Packet unexpected length.

void USBSendError(
    uint8_t deviceId,
    ComsErrorTypeT code,
    uint8_t originalPacketType,
    uint8_t data
    )
{
  PacketErrorC pkt;
  pkt.m_packetType = CPT_Error;
  pkt.m_deviceId = deviceId;
  pkt.m_errorCode = code;
  pkt.m_causeType = originalPacketType;
  pkt.m_errorData = data;

  USBSendPacket((uint8_t *) &pkt,sizeof(struct PacketErrorC));
}



uint8_t g_debugIndex = 0x55;

bool SetParam(enum ComsParameterIndexT index,union BufferTypeT *dataBuff,int len)
{
  switch(index )
  {
    case CPI_FirmwareVersion:
      return false; // Can't set this
    case CPI_PWMState:
      if(len < 1)
        return false;
      if(dataBuff->uint8[0] > 0) {
        PWMRun();
      } else {
        PWMStop();
      }
      break;
    case CPI_PWMMode:
      if(len != 1)
        return false;
      if(dataBuff->uint8[0] >= (int) CM_Final)
        return false;
      g_controlMode = (PWMControlDynamicT) dataBuff->uint8[0];
      break;
    case CPI_PWMFullReport:
      if(len != 1)
        return false;
      g_pwmFullReport = dataBuff->uint8[0] > 0;
      break;
    case CPI_CANBridgeMode:
      if(len != 1)
        return false;
      g_canBridgeMode = dataBuff->uint8[0] > 0;
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

    case CPI_AuxPower:
      if(len != 1)
        return false;
      if(dataBuff->uint8[0] > 0)
        palSetPad(GPIOA, GPIOA_PIN7);
      else
        palClearPad(GPIOA, GPIOA_PIN7);
      return true;

    case CPI_CalibrationOffset:
      if(len != 4)
        return false;
      g_homeAngleOffset = dataBuff->float32[0] * g_actuatorRatio;
      return true;

    case CPI_HomedState: {
      if(len != 1)
        return false;
      enum MotionHomedStateT newCal = (enum MotionHomedStateT) dataBuff->uint8[0];
      switch(newCal)
      {
        case MHS_Lost:
          MotionResetCalibration(newCal);
          break;
        case MHS_Measuring:
          MotionResetCalibration(MHS_Measuring);
          break;
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
      enum PositionReferenceT posRef = (enum PositionReferenceT) dataBuff->uint8[0];
      switch(posRef)
      {
        case PR_Relative:
        case PR_Absolute:
        case PR_OtherJointRelative:
        case PR_OtherJointAbsolute:
          g_motionPositionReference = posRef;
          break;
        default:
          return false;
      }
    } break;
    case CPI_ControlState: {
      if(len != 1)
        return false;
      enum ControlStateT newState = (enum ControlStateT) dataBuff->uint8[0];
      if(!ChangeControlState(newState))
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
      g_indicatorState = dataBuff->uint8[0] > 0;
      break;
    case CPI_OtherJoint:
      if(len != 1)
        return false;
      g_otherJointId = dataBuff->uint8[0];
      break;
    case CPI_OtherJointGain:
      if(len != 4)
        return false;
      g_relativePositionGain = dataBuff->float32[0];
      break;
    case CPI_OtherJointOffset:
      if(len != 4)
        return false;
      g_relativePositionOffset = dataBuff->float32[0] * g_actuatorRatio;
      break;
    case CPI_DebugIndex:
      g_debugIndex = len;
      break;
    case CPI_MotorIGain:
      if(len != 4)
        return false;
      g_motor_i_gain = dataBuff->float32[0];
      break;
    case CPI_MotorPGain:
      if(len != 4)
        return false;
      g_motor_p_gain = dataBuff->float32[0];
      break;
    case CPI_VelocityPGain:
      if(len != 4)
        return false;
      g_velocityPGain =  dataBuff->float32[0];
      break;
    case CPI_VelocityIGain:
      if(len != 4)
        return false;
      g_velocityIGain =  dataBuff->float32[0];
      break;
    case CPI_DemandPhaseVelocity:
      if(len != 4)
        return false;
      g_demandPhaseVelocity = dataBuff->float32[0];
      break;
    case CPI_VelocityLimit:
      if(len != 4)
        return false;
      g_velocityLimit = dataBuff->float32[0];
      break;
    case CPI_PositionGain:
      if(len != 4)
        return false;
      g_positionGain = dataBuff->float32[0];
      break;
    case CPI_MaxCurrent:
      if(len != 4)
        return false;
      g_absoluteMaxCurrent = dataBuff->float32[0];
      break;
    case CPI_homeIndexPosition:
      if(len != 4)
        return false;
      g_homeIndexPosition = dataBuff->float32[0];
      break;
    case CPI_MinSupplyVoltage:
      if(len != 4)
        return false;
      g_minSupplyVoltage = dataBuff->float32[0];
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
        g_phaseAngles[reg][i] = dataBuff->uint16[i];
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
      data->uint8[0] = (int) DT_MotorDriver;
      break;
    case CPI_FirmwareVersion:
      *len = 1;
      data->uint8[0] = 2;
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

    case CPI_AuxPower: {
      *len = 1;
      data->uint8[0] = palReadPad(GPIOA,GPIOA_PIN7);
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
    case CPI_OtherJoint:
      *len = 1;
      data->uint8[0] = g_otherJointId;
      break;
    case CPI_OtherJointGain:
      *len = 4;
      data->float32[0] = g_relativePositionGain;
      break;
    case CPI_OtherJointOffset:
      *len = 4;
      data->float32[0] = g_relativePositionOffset / g_actuatorRatio;
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
    default:
      return false;
  }
  return true;
}

bool USBReadParamAndReply(enum ComsParameterIndexT paramIndex)
{
  struct PacketParam8ByteC reply;
  reply.m_header.m_packetType = CPT_ReportParam;
  reply.m_header.m_deviceId = g_deviceId;
  reply.m_header.m_index = paramIndex;
  int len = 0;
  if(!ReadParam(paramIndex,&len,&reply.m_data))
    return false;

  USBSendPacket((uint8_t *)&reply,sizeof(reply.m_header) + len);
  return true;
}


void SendParamUpdate(enum ComsParameterIndexT paramIndex) {
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
}

//! Process received packet.

void ProcessPacket(const uint8_t *m_data,int m_packetLen)
{
  if(m_packetLen < 1)
    return ;
  enum ComsPacketTypeT cpt = (enum ComsPacketTypeT) m_data[0];
  switch(cpt)
  {
  case CPT_NoOp: // No-op
    break;
  case CPT_EmergencyStop:
    ChangeControlState(CS_EmergencyStop);
    if(!CANEmergencyStop()) {
      USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_EmergencyStop,0);
      // Retry ?
    }
    break;
  case CPT_Ping: { // Ping.
    if(m_packetLen != sizeof(struct PacketPingPongC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,CPT_Ping,m_data[0]);
      break;
    }
    const PacketPingPongC *pkt = (const PacketPingPongC *) m_data;
    if(pkt->m_deviceId == g_deviceId || pkt->m_deviceId == 0) {
      // If it is for this node, just reply
      PacketPingPongC pkt;
      pkt.m_packetType = CPT_Pong;
      pkt.m_deviceId = g_deviceId;
      USBSendPacket((uint8_t *) &pkt,sizeof(struct PacketPingPongC));
    }
    if((pkt->m_deviceId != g_deviceId || pkt->m_deviceId == 0) && g_deviceId != 0) {
      // In bridge mode forward this to the CAN bus
      if(g_canBridgeMode) {
        CANPing(CPT_Ping,pkt->m_deviceId);
      }
    }
  } break;
  case CPT_BridgeMode: {
    if(m_packetLen != sizeof(struct PacketBridgeModeC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,CPT_BridgeMode,m_packetLen);
      break;
    }
    struct PacketBridgeModeC *psp = (struct PacketBridgeModeC *) m_data;
    g_canBridgeMode = psp->m_enable;
  } break;
  case CPT_Pong: break; // Ping reply.
  case CPT_Sync: break; // Sync.
  case CPT_Error: break; // Error.
  case CPT_ReportParam: break;
  case CPT_PWMState: break; // Error.
  case CPT_ReadParam: {
    if(m_packetLen != sizeof(struct PacketReadParamC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,CPT_ReadParam,m_packetLen);
      break;
    }
    struct PacketReadParamC *psp = (struct PacketReadParamC *) m_data;
    if(psp->m_deviceId == g_deviceId || psp->m_deviceId == 0) {
      if(!USBReadParamAndReply((enum ComsParameterIndexT) psp->m_index)) {
        USBSendError(g_deviceId,CET_ParameterOutOfRange,CPT_ReadParam,psp->m_index);
      }
    }
    if((psp->m_deviceId != g_deviceId || psp->m_deviceId == 0) && g_deviceId != 0) {
      if(!CANSendReadParam(psp->m_deviceId,psp->m_index)) {
        USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_ReadParam,psp->m_index);
      }
    }
  } break;
  case CPT_SetParam: {
    struct PacketParam8ByteC *psp = (struct PacketParam8ByteC *) m_data;
    if(m_packetLen < (int) sizeof(psp->m_header)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,CPT_SetParam,m_packetLen);
      break;
    }
    int dataLen = m_packetLen-sizeof(psp->m_header);
    if(psp->m_header.m_deviceId == g_deviceId || psp->m_header.m_deviceId == 0) {
      if(!SetParam((enum ComsParameterIndexT) psp->m_header.m_index,&psp->m_data,dataLen)) {
        USBSendError(g_deviceId,CET_ParameterOutOfRange,CPT_SetParam,psp->m_header.m_index);
      }
    }
    if(g_canBridgeMode &&
        (psp->m_header.m_deviceId != g_deviceId  || psp->m_header.m_deviceId == 0) &&
        g_deviceId != 0)
    {
      if(!CANSendSetParam(psp->m_header.m_deviceId,psp->m_header.m_index,&psp->m_data,dataLen)) {
        USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_SetParam,psp->m_header.m_index);
      }
    }
  } break;
  case CPT_ServoReport: break; // Drop
  case CPT_Servo: { // Goto position.
    if(m_packetLen != sizeof(struct PacketServoC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,CPT_Servo,m_packetLen);
      break;
    }
    PacketServoC *ps = (PacketServoC *) m_data;
    if(ps->m_deviceId == g_deviceId || ps->m_deviceId == 0) {
      MotionSetPosition(ps->m_mode,ps->m_position,ps->m_torqueLimit);
    } else {
      if(g_canBridgeMode && g_deviceId != 0) {
        CANSendServo(ps->m_deviceId,ps->m_position,ps->m_torqueLimit,ps->m_mode);
      }
    }
  } break;
  case CPT_QueryDevices: {
    // First report about myself
    struct PacketDeviceIdC pkt;
    pkt.m_packetType = CPT_AnnounceId;
    pkt.m_deviceId = g_deviceId;
    pkt.m_uid[0] = g_nodeUId[0];
    pkt.m_uid[1] = g_nodeUId[1];
    USBSendPacket((uint8_t *) &pkt,sizeof(struct PacketDeviceIdC));

    // Are we bridging ?
    if(g_canBridgeMode) {
      if(!CANSendQueryDevices())
        USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_QueryDevices,0);
    }
  } break;
  case CPT_AnnounceId: break; // Drop
  case CPT_SetDeviceId: {
    if(m_packetLen != sizeof(struct PacketDeviceIdC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,CPT_SetDeviceId,m_packetLen);
      break;
    }
    const struct PacketDeviceIdC *pkt = (const struct PacketDeviceIdC *) m_data;
    // Check if we're setting the id for this device.
    if(pkt->m_uid[0] == g_nodeUId[0] &&
        pkt->m_uid[1] == g_nodeUId[1]) {
      g_deviceId = pkt->m_deviceId;

      // Announce change.
      {
        struct PacketDeviceIdC rpkt;
        rpkt.m_packetType = CPT_AnnounceId;
        rpkt.m_deviceId = g_deviceId;
        rpkt.m_uid[0] = g_nodeUId[0];
        rpkt.m_uid[1] = g_nodeUId[1];
        USBSendPacket((uint8_t *) &rpkt,sizeof(struct PacketDeviceIdC));
      }
    } else {
      if(g_canBridgeMode) {
        CANSendSetDevice(pkt->m_deviceId,pkt->m_uid[0],pkt->m_uid[1]);
      }
    }
  } break;
  case CPT_LoadSetup:
  case CPT_SaveSetup: { // Save setup from eeprom
    if(m_packetLen != sizeof(struct PacketStoredConfigC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,CPT_SaveSetup,m_packetLen);
      break;
    }
    const struct PacketStoredConfigC *psp = (const struct PacketStoredConfigC *) m_data;
    if(psp->m_deviceId == g_deviceId || psp->m_deviceId == 0) {
      enum FaultCodeT ret;
      // Having to do this suggests there should be a way of refactoring.
      switch(cpt) {
      case CPT_SaveSetup: ret = SaveSetup(); break;
      case CPT_LoadSetup: ret = LoadSetup(); break;
      default: ret = FC_Internal; break;
      }
      if(ret != FC_Ok) {
        USBSendError(g_deviceId,CET_InternalError,CPT_LoadSetup,ret);
      }
    }
    if((psp->m_deviceId != g_deviceId || psp->m_deviceId == 0) && g_deviceId != 0) {
      if(!CANSendStoredSetup(psp->m_deviceId,cpt)) {
        USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_SaveSetup,m_data[0]);
      }
    }
  } break;
  case CPT_CalZero: {
    if(m_packetLen != sizeof(struct PacketCalZeroC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,CPT_CalZero,m_packetLen);
      break;
    }
    struct PacketCalZeroC *psp = (struct PacketCalZeroC *) m_data;
    if(psp->m_deviceId == g_deviceId || psp->m_deviceId == 0) {
      if(!MotionCalZero()) {
        USBSendError(g_deviceId,CET_MotorNotRunning,CPT_CalZero,0);
      }
    }
    if((psp->m_deviceId != g_deviceId || psp->m_deviceId == 0) && g_deviceId != 0) {
      if(!CANSendCalZero(psp->m_deviceId)) {
        USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_CalZero,m_data[0]);
      }
    }

  } break;
  case CPT_SyncTime:
    // Unsupported yet.
    USBSendError(g_deviceId,CET_UnknownPacketType,m_data[0],m_packetLen);
    break;
#if 1
  default: {
    USBSendError(g_deviceId,CET_UnknownPacketType,m_data[0],m_packetLen);
    //RavlDebug("Unexpected packet type %d ",(int) m_data[1]);
  } break;
#endif
  }
}



