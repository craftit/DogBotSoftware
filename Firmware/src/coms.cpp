
#include <stdint.h>

#include "hal.h"
#include "coms.h"
#include "canbus.h"
#include "dogbot/protocol.h"
#include "bmc.h"
#include "motion.h"
#include "flashops.hh"

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

bool USBSendPing(uint8_t deviceId,uint16_t payload)
{
  PacketPingPongC pkt;
  pkt.m_packetType = CPT_Ping;
  pkt.m_deviceId = deviceId;
  pkt.m_payload = payload;
  return USBSendPacket((uint8_t *) &pkt,sizeof(struct PacketPingPongC));
}

bool USBSendBootLoaderResult(uint8_t deviceId,uint8_t lastSeqNum,enum BootLoaderStateT state,enum FlashOperationStatusT result)
{
  struct PacketFlashResultC pkt;
  pkt.m_packetType = CPT_FlashCmdResult;
  pkt.m_deviceId = deviceId;
  pkt.m_rxSequence = lastSeqNum;
  pkt.m_state = state;
  pkt.m_result = result;
  return USBSendPacket((uint8_t *) &pkt,sizeof(pkt));
}

bool USBSendBootLoaderCheckSumResult(uint8_t deviceId,uint8_t seqNum,uint32_t sum)
{
  struct PacketFlashChecksumResultC pkt;
  pkt.m_packetType = CPT_FlashChecksumResult;
  pkt.m_deviceId = deviceId;
  pkt.m_sequenceNumber = seqNum;
  pkt.m_sum = sum;
  return USBSendPacket((uint8_t *) &pkt,sizeof(pkt));
}

bool USBSendBootLoaderData(uint8_t deviceId,uint8_t seqNum,uint8_t *data,uint8_t len)
{
  struct PacketFlashDataBufferC pkt;
  pkt.m_header.m_packetType = CPT_FlashData;
  pkt.m_header.m_deviceId = deviceId;
  pkt.m_header.m_sequenceNumber = seqNum;
  int dataLen = len;
  memcpy(pkt.m_data,data,dataLen);
  return USBSendPacket((uint8_t *) &pkt,sizeof(pkt.m_header)+dataLen);
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


/* Report an error  */
void SendError(enum ComsErrorTypeT code,uint8_t originalPacketType,uint8_t data)
{
  if(g_canBridgeMode) {
    USBSendError(g_deviceId,code,originalPacketType,data);
  }
  CANSendError(code,originalPacketType,data);
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
  case CPT_EmergencyStop: {
      enum StateChangeSourceT cause = SCS_Unknown;
      uint8_t sourceDeviceId = 0;
      if(m_packetLen == sizeof(PacketEmergencyStopC)) {
        const PacketEmergencyStopC *pkt = (const PacketEmergencyStopC *) m_data;
        sourceDeviceId = pkt->m_deviceId;
        cause = (enum StateChangeSourceT) pkt->m_cause;
      }
      ChangeControlState(CS_EmergencyStop,cause);
      if(!CANEmergencyStop(sourceDeviceId,cause)) {
        USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_EmergencyStop,0);
        // It would be nice to retry, but we don't know where this is called
        // from and if a delay is safe.
      }
    } break;
  case CPT_Ping: { // Ping.
    if(m_packetLen != sizeof(struct PacketPingPongC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,CPT_Ping,m_data[0]);
      break;
    }
    const PacketPingPongC *pkt = (const PacketPingPongC *) m_data;
    if(pkt->m_deviceId == g_deviceId || pkt->m_deviceId == 0) {
      // If it is for this node, just reply
      PacketPingPongC replyPkt;
      replyPkt.m_packetType = CPT_Pong;
      replyPkt.m_deviceId = g_deviceId;
      replyPkt.m_payload = pkt->m_payload;
      USBSendPacket((uint8_t *) &replyPkt,sizeof(struct PacketPingPongC));
    }
    if((pkt->m_deviceId != g_deviceId || pkt->m_deviceId == 0) && g_deviceId != 0) {
      // In bridge mode forward this to the CAN bus
      if(g_canBridgeMode) {
        if(!CANPing(CPT_Ping,pkt->m_deviceId,pkt->m_payload)) {
          USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_Ping,pkt->m_deviceId);
        }
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
  case CPT_Sync: // Sync communications.
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
        if(!CANSendServo(ps->m_deviceId,ps->m_position,ps->m_torqueLimit,ps->m_mode)) {
          USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_Servo,ps->m_deviceId);
        }
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
        if(!CANSendSetDevice(pkt->m_deviceId,pkt->m_uid[0],pkt->m_uid[1])) {
          USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_SetDeviceId,pkt->m_deviceId);
        }
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
    if(g_canBridgeMode) {
      if((psp->m_deviceId != g_deviceId || psp->m_deviceId == 0) && g_deviceId != 0) {
        if(!CANSendStoredSetup(psp->m_deviceId,cpt)) {
          USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_SaveSetup,m_data[0]);
        }
      }
    }
  } break;
  case CPT_CalZero: {
    if(m_packetLen != sizeof(struct PacketCalZeroC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,cpt,m_packetLen);
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
    if(g_canBridgeMode) {
      MotionSyncTime();
      if(!CANSyncTime()) {
        USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_SyncTime,0);
      }
    }
    break;
  case CPT_FlashCmdReset: { // Status from a flash command
    if(m_packetLen != sizeof(struct PacketFlashResetC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,cpt,m_packetLen);
      break;
    }
    struct PacketFlashResetC *psp = (struct PacketFlashResetC *) m_data;
    if(psp->m_deviceId == g_deviceId || psp->m_deviceId == 0) {
      BootLoaderReset(psp->m_enable > 0);
    }
    if(g_canBridgeMode) {
      if(psp->m_deviceId != g_deviceId || psp->m_deviceId == 0) {
        if(!CANSendBootLoaderReset(psp->m_deviceId,psp->m_enable > 0)) {
          USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_FlashCmdReset,psp->m_deviceId);
        }
      }
    }
  } break;

  case CPT_FlashCmdResult:  { // Status from a flash command
    if(m_packetLen != sizeof(struct PacketFlashResultC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,cpt,m_packetLen);
      break;
    }
    struct PacketFlashResultC *psp = (struct PacketFlashResultC *) m_data;
    if(g_canBridgeMode) {
      if(psp->m_deviceId != g_deviceId || psp->m_deviceId == 0) {
        if(!CANSendBootLoaderResult(psp->m_deviceId,psp->m_rxSequence,(enum BootLoaderStateT) psp->m_state,(enum FlashOperationStatusT) psp->m_result)) {
          USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_FlashCmdResult,psp->m_deviceId);
        }
      }
    }
  } break;
  case CPT_FlashChecksumResult: {// Generate a checksum
    if(m_packetLen != sizeof(struct PacketFlashChecksumResultC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,cpt,m_packetLen);
      break;
    }
    auto *psp = (struct PacketFlashChecksumResultC *) m_data;
    if(g_canBridgeMode) {
      if(psp->m_deviceId != g_deviceId || psp->m_deviceId == 0) {
        if(!CANSendBootLoaderCheckSumResult(psp->m_deviceId,psp->m_sequenceNumber,psp->m_sum)) {
          USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_FlashChecksumResult,psp->m_deviceId);
        }
      }
    }
  } break;
  case CPT_FlashEraseSector: { // Erase a flash sector
    if(m_packetLen != sizeof(struct PacketFlashEraseC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,cpt,m_packetLen);
      break;
    }
    auto *psp = (struct PacketFlashEraseC *) m_data;
    if(psp->m_deviceId == g_deviceId || psp->m_deviceId == 0) {
      BootLoaderErase(psp->m_sequenceNumber,psp->m_addr);
    }
    if(g_canBridgeMode) {
      if(psp->m_deviceId != g_deviceId || psp->m_deviceId == 0) {
        if(!CANSendBootLoaderErase(psp->m_deviceId,psp->m_sequenceNumber,psp->m_addr)) {
          USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_FlashEraseSector,psp->m_deviceId);
        }
      }
    }
  } break;
  case CPT_FlashChecksum: { // Generate a checksum
    if(m_packetLen != sizeof(struct PacketFlashChecksumC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,cpt,m_packetLen);
      break;
    }
    auto *psp = (struct PacketFlashChecksumC *) m_data;
    if(psp->m_deviceId == g_deviceId || psp->m_deviceId == 0) {
      BootLoaderCheckSum(psp->m_sequenceNumber,psp->m_addr,psp->m_len);
    }
    if(g_canBridgeMode) {
      if(psp->m_deviceId != g_deviceId || psp->m_deviceId == 0) {
        if(!CANSendBootLoaderCheckSum(psp->m_deviceId,psp->m_sequenceNumber,psp->m_addr,psp->m_len)) {
          USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_FlashChecksum,psp->m_deviceId);
        }
      }
    }
  } break;
  case CPT_FlashData: { // Data packet
    if(m_packetLen < (int) sizeof(struct PacketFlashDataC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,cpt,m_packetLen);
      break;
    }
    auto *psp = (struct PacketFlashDataC *) m_data;
    int len = m_packetLen - sizeof(struct PacketFlashDataC);
    if(psp->m_deviceId == g_deviceId || psp->m_deviceId == 0) {
      BootLoaderData(psp->m_sequenceNumber,psp->m_data,len);
    }
    if(g_canBridgeMode) {
      if(psp->m_deviceId != g_deviceId || psp->m_deviceId == 0) {
        if(!CANSendBootLoaderData(psp->m_deviceId,psp->m_sequenceNumber,&psp->m_data[0],len)) {
          USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_FlashData,psp->m_deviceId);
        }
      }
    }
  } break;
  case CPT_FlashWrite: {// Write buffer
    if(m_packetLen != sizeof(struct PacketFlashWriteC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,cpt,m_packetLen);
      break;
    }
    auto *psp = (struct PacketFlashWriteC *) m_data;
    if(psp->m_deviceId == g_deviceId || psp->m_deviceId == 0) {
      BootLoaderBeginWrite(psp->m_sequenceNumber,psp->m_addr,psp->m_len);
    }
    if(g_canBridgeMode) {
      if(psp->m_deviceId != g_deviceId || psp->m_deviceId == 0) {
        if(!CANSendBootLoaderWrite(psp->m_deviceId,psp->m_sequenceNumber,psp->m_addr,psp->m_len)) {
          USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_FlashWrite,psp->m_deviceId);
        }
      }
    }

  } break;
  case CPT_FlashRead: { // Read buffer and send it back
    if(m_packetLen != sizeof(struct PacketFlashReadC)) {
      USBSendError(g_deviceId,CET_UnexpectedPacketSize,cpt,m_packetLen);
      break;
    }
    auto *psp = (struct PacketFlashReadC *) m_data;
    if(psp->m_deviceId == g_deviceId || psp->m_deviceId == 0) {
      BootLoaderBeginRead(psp->m_sequenceNumber,psp->m_addr,psp->m_len);
    }
    if(g_canBridgeMode) {
      if(psp->m_deviceId != g_deviceId || psp->m_deviceId == 0) {
        if(!CANSendBootLoaderRead(psp->m_deviceId,psp->m_sequenceNumber,psp->m_addr,psp->m_len)) {
          USBSendError(g_deviceId,CET_CANTransmitFailed,CPT_FlashRead,psp->m_deviceId);
        }
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



