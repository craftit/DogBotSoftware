
#include "serial_packet.hh"


void SerialDecodeC::AcceptByte(uint8_t sendByte)
{
  switch(m_state)
  {
  default: // This can only be caused by a bug.
    g_usbErrorCount++;
    m_state = 0;
    /* no break */
  case 0: // Wait for STX.
    if(sendByte == g_charSTX)
      m_state = 1;
    // Else remain in state 0.
    break;
  case 1: // Packet length.
    m_packetLen = sendByte;
    if(m_packetLen >= m_maxPacketSize) {
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

//! Process received packet.

void SerialDecodeC::ProcessPacket()
{
  enum ComsPacketTypeT cpt = (enum ComsPacketTypeT) m_data[0];
  switch(cpt)
  {
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

SerialDecodeC g_comsDecode;


