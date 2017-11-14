
#include <stdint.h>

#include "coms.h"
#include "canbus.h"
#include "dogbot/protocol.h"
#include "chmboxes.h"
#include "pwm.h"
#include "mathfunc.h"
#include "motion.h"
#include "drv8503.h"

#include <string.h>

const uint8_t g_charSTX = 0x02;
const uint8_t g_charETX = 0x03;

bool g_canBridgeMode = false;
bool g_comsInitDone = false;

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


class SerialDecodeC
{
public:

  //! Accept a byte
  void AcceptByte(uint8_t sendByte);

  //! Process received packet.
  void ProcessPacket();

  int m_state = 0;
  int m_checkSum = 0;
  int m_packetLen = 0;
  uint8_t m_data[255];
  int m_at = 0;

  BaseSequentialStream  *m_SDU = 0;

  // Packet structure.
  // x    STX
  // x    Len - Of data excluding STX,ETX and Checksum.
  // 0    Address
  // 1    Type
  // 2..n data.
  // n    2-CRC
  // n    ETX.
} g_comsDecode;

void SerialDecodeC::AcceptByte(uint8_t sendByte)
{
  switch(m_state)
  {
  case 0: // Wait for STX.
    if(sendByte == g_charSTX)
      m_state = 1;
    // Else remain in state 0.
    break;
  case 1: // Packet length.
    m_packetLen = sendByte;
    if(m_packetLen > 64) {
      if(sendByte != g_charSTX) // This will always be false, but for good form.
        m_state = 0;
      break;
    }

    m_at = 0;
    m_checkSum = 0x55 + m_packetLen;
    m_state = 2;
    break;
  case 2: // Data
    m_checkSum += sendByte;
    m_data[m_at] = sendByte;
    m_at++;
    if(m_at >= m_packetLen)
      m_state = 3;
    break;
  case 3: { // CRC 1
    uint8_t cs1 = (m_checkSum & 0xff);
    //RavlDebug("Checksum1 : %d %d ",(int)  cs1 , (int) sendByte);
    if(cs1 != sendByte) {
      //RavlDebug("Checksum failed. ");
      if(sendByte == g_charSTX)
        m_state = 1;
      else
        m_state = 0;
      break;
    }

    m_state = 4;
  } break;
  case 4: { // CRC 2
    uint8_t cs2 = ((m_checkSum >> 8) & 0xff);
    //RavlDebug("Checksum2 : %d %d ",(int) ((m_checkSum >> 8) & 0xff) , (int) sendByte);
    if(cs2 != sendByte) {
      //RavlDebug("Checksum failed. ");
      if(sendByte == g_charSTX)
        m_state = 1;
      else
        m_state = 0;
      break;
    }

    m_state = 5;
  } break;
  case 5: // ETX.
    if(sendByte == g_charETX) {
      //RavlDebug("Got packet!");
      ProcessPacket();
    } else {
      // FIXME Count corrupted packets ?
      //RavlDebug("Packet corrupted.");
      if(sendByte == g_charSTX) {
        m_state = 1;
        break;
      }
    }
    m_state = 0;
    break;
  }
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
      g_controlMode = (PWMControlModeT) dataBuff->uint8[0];
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
      g_angleOffset = dataBuff->float32[0] * g_actuatorRatio;
      return true;

    case CPI_PositionCal: {
      if(len != 1)
        return false;
      enum MotionCalibrationT newCal = (enum MotionCalibrationT) dataBuff->uint8[0];
      switch(newCal)
      {
        case MC_Uncalibrated:
          MotionResetCalibration(newCal);
          break;
        case MC_Measuring:
          MotionResetCalibration(MC_Measuring);
          break;
        case MC_Calibrated:
          // Let user know it hasn't changed.
          if(g_motionCalibration  != newCal)
            SendParamUpdate(CPI_PositionCal);
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
    case CPI_ANGLE_CAL_11: {
      int reg = ((int) index - CPI_ANGLE_CAL);
      if(len != 6) return false;
      for(int i = 0;i < 3;i++)
        g_phaseAngles[reg][i] = dataBuff->uint16[i];
    } break;
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
      data->uint8[0] = 1;
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
    case CPI_PositionCal:
      *len = 1;
      data->uint8[0] = (int) g_motionCalibration;
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
      data->float32[0] = g_angleOffset / g_actuatorRatio;
      break;
    case CPI_DriveTemp:
      *len = 4;
      data->float32[0] = g_driveTemperature;
      break;
    case CPI_MotorTemp:
      *len = 4;
      data->float32[0] = 0;
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
    case CPI_ANGLE_CAL_11: {
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
void SerialDecodeC::ProcessPacket()
{
  enum ComsPacketTypeT cpt = (enum ComsPacketTypeT) m_data[0];
  switch(cpt)
  {
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

  case CPT_Pong: break; // Ping reply.
  case CPT_Sync: break; // Sync.
  case CPT_Error: break; // Error.
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
#if 1
  default: {
    USBSendError(g_deviceId,CET_UnknownPacketType,m_data[0],m_packetLen);
    //RavlDebug("Unexpected packet type %d ",(int) m_data[1]);
  } break;
#endif
  }
}

#define PACKET_QUEUE_SIZE 8

static msg_t g_emptyPacketData[PACKET_QUEUE_SIZE];
static mailbox_t g_emptyPackets;
static msg_t g_fullPacketData[PACKET_QUEUE_SIZE];
static mailbox_t g_fullPackets;

static PacketT g_packetArray[PACKET_QUEUE_SIZE];


/* Get a free packet structure. */
struct PacketT *GetEmptyPacket(systime_t timeout)
{
  if(!g_comsInitDone)
    return 0;

  msg_t msg;
  if(chMBFetch(&g_emptyPackets,&msg,timeout) != MSG_OK)
    return 0;
  return (PacketT *)msg;
}

/* Post packet. */
bool PostPacket(struct PacketT *pkt)
{
  if(!g_comsInitDone)
    return false;

  if(chMBPost(&g_fullPackets,(msg_t) pkt,TIME_IMMEDIATE) == MSG_OK)
    return true;

  // This shouldn't happen, as if we can't acquire an empty buffer
  // unless there is space, but we don't want to loose the buffer
  // so attempt to add it back to the free list
  if(chMBPost(&g_emptyPackets,(msg_t) pkt,TIME_IMMEDIATE) != MSG_OK) {
    // Things are screwy. Panic ?
  }
  return false;
}



static THD_WORKING_AREA(waThreadRxComs, 512);
static THD_FUNCTION(ThreadRxComs, arg) {

  (void)arg;
  chRegSetThreadName("rxcoms");
  while(true) {
    int ret = streamGet(g_comsDecode.m_SDU);
    if(ret == STM_RESET) {
      chThdSleepMilliseconds(500);
      continue;
    }
    g_comsDecode.AcceptByte(ret);
  }
}

static THD_WORKING_AREA(waThreadTxComs, 512);
static THD_FUNCTION(ThreadTxComs, arg) {

  (void)arg;
  chRegSetThreadName("txcoms");
  static uint8_t txbuff[261];
  while(true) {
    struct PacketT *packet = 0;
    if(chMBFetch(&g_fullPackets,(msg_t*) &packet,TIME_INFINITE) != MSG_OK)
      continue;

    if(SDU1.config->usbp->state == USB_ACTIVE) {

      uint8_t len = packet->m_len;
      uint8_t *buff = packet->m_data;

      uint8_t *at = txbuff;
      *(at++) = g_charSTX;
      *(at++) = len;
      int crc = len + 0x55;
      for(int i = 0;i < len;i++) {
        uint8_t data = buff[i];
        *(at++) = data;
        crc += data;
      }
      *(at++) = crc;
      *(at++) = crc>>8;
      *(at++) = g_charETX;
      int size = at - txbuff;

      chnWrite(g_packetStream, txbuff, size);
    } else {
      // If we've lost our connection turn bridge mode off.
      g_canBridgeMode = false;
    }

    chMBPost(&g_emptyPackets,reinterpret_cast<msg_t>(packet),TIME_INFINITE);
  }
}


bool USBSendPacket(
    uint8_t *buff,
    int len
    )
{
  // Just truncate for the moment, it is better than overwriting memory.
  struct PacketT *pkt;
  //if(len > sizeof pkt->m_data) len = sizeof pkt->m_data;

  if((pkt = GetEmptyPacket(TIME_IMMEDIATE)) == 0)
    return false;

  pkt->m_len = len;
  memcpy(&pkt->m_data,buff,len);
  PostPacket(pkt);

  return true;
}

BaseSequentialStream *g_packetStream = 0;

void InitComs()
{
  if(g_comsInitDone)
    return ;

  g_comsDecode.m_SDU = (BaseSequentialStream *)&SDU1;
  g_packetStream = (BaseSequentialStream *)&SDU1;

  chMBObjectInit(&g_emptyPackets,g_emptyPacketData,PACKET_QUEUE_SIZE);
  for(int i = 0;i < PACKET_QUEUE_SIZE;i++) {
    chMBPost(&g_emptyPackets,reinterpret_cast<msg_t>(&g_packetArray[i]),TIME_IMMEDIATE);
  }
  chMBObjectInit(&g_fullPackets,g_fullPacketData,PACKET_QUEUE_SIZE);

  g_comsInitDone = true;

  chThdCreateStatic(waThreadTxComs, sizeof(waThreadTxComs), NORMALPRIO, ThreadTxComs, NULL);
  chThdCreateStatic(waThreadRxComs, sizeof(waThreadRxComs), NORMALPRIO, ThreadRxComs, NULL);

}

