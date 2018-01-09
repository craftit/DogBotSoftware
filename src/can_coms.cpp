
#include "coms.h"
#include "motion.h"
#include "pwm.h"
#include "canbus.h"
#include "can_coms.hh"
#include <string.h>
#include "can_queue.hh"

bool CANSendStoredSetup(
    uint8_t deviceId,
    enum ComsPacketTypeT pktType // Must be either CPT_SaveSetup or CPT_LoadSetup
)
{
  switch(pktType)
  {
    case CPT_SaveSetup:
    case CPT_LoadSetup:
      break;
    default:
      return false;
  }
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,pktType);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 0;
  return g_txCANQueue.PostFullPacket(txmsg);
}


bool CANSendCalZero(
    uint8_t deviceId
    )
{
  enum ComsPacketTypeT pktType = CPT_CalZero;

  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,pktType);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 0;
  return g_txCANQueue.PostFullPacket(txmsg);
}


bool CANSendServoReport(
    uint8_t deviceId,
    int16_t position,
    int16_t torque,
    uint8_t state
    )
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,CPT_ServoReport);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 5;
  txmsg->data16[0] = position;
  txmsg->data16[1] = torque;
  txmsg->data8[4] = state;
  return g_txCANQueue.PostFullPacket(txmsg);
}

bool CANSendServo(
    uint8_t deviceId,
    int16_t position,
    uint16_t torqueLimit,
    uint8_t state
    )
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;

  CANSetAddress(txmsg,deviceId,CPT_Servo);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 5;
  txmsg->data16[0] = position;
  txmsg->data16[1] = torqueLimit;
  txmsg->data8[4] = state;
  return g_txCANQueue.PostFullPacket(txmsg);
}

// Code for processing incoming CAN packets.

bool CANRecieveFrame(CANRxFrame *rxmsgptr)
{
  CANRxFrame &rxmsg = *rxmsgptr;

  //palTogglePad(GPIOC, GPIOC_PIN5);       /* Yellow led. */

  int rxDeviceId = rxmsg.SID  & CAN_MSG_NODE_MASK;
  int msgType =  (rxmsg.SID >> CAN_MSG_TYPEBIT) & CAN_MSG_TYPEMASK;

  switch((enum ComsPacketTypeT) msgType)
  {
    case CPT_NoOp:
      break;
    case CPT_EmergencyStop: { // Ping request
      ChangeControlState(CS_EmergencyStop);
      if(g_canBridgeMode) {
        uint8_t pktType = CPT_EmergencyStop;
        USBSendPacket((uint8_t *) &pktType,sizeof(pktType));
      }
    } break;
    case CPT_Ping: { // Ping request
      if(g_deviceId == rxDeviceId || rxDeviceId == 0) {
        if(rxmsg.DLC != 0) {
          CANSendError(CET_UnexpectedPacketSize,CPT_Ping,rxmsg.DLC);
          break;
        }
        CANPing(CPT_Pong,g_deviceId);
      }
    } break;
    case CPT_Pong: {
      if(g_canBridgeMode) {
        if(rxmsg.DLC != 0) {
          USBSendError(rxDeviceId,CET_UnexpectedPacketSize,CPT_Pong,rxmsg.DLC);
          break;
        }
        struct PacketPingPongC pkt;
        pkt.m_packetType = CPT_Pong;
        pkt.m_deviceId = rxDeviceId;
        USBSendPacket((uint8_t *) &pkt,sizeof(struct PacketPingPongC));
      }
    } break;
    case CPT_Error:
      if(g_canBridgeMode) {
        if(rxmsg.DLC != 3) {
          USBSendError(rxDeviceId,CET_UnexpectedPacketSize,CPT_Error,rxmsg.DLC);
          break;
        }
        struct PacketErrorC pkt;
        pkt.m_packetType = CPT_Error;
        pkt.m_deviceId = rxDeviceId;
        pkt.m_errorCode = rxmsg.data8[0];
        pkt.m_causeType = rxmsg.data8[1];
        pkt.m_errorData = rxmsg.data8[2];
        USBSendPacket((uint8_t *) &pkt,sizeof(struct PacketErrorC));
      }
      break;
    case CPT_Sync:
    case CPT_PWMState:
    {
      // Drop it.
    } break;
    case CPT_ReportParam: {
      if(g_canBridgeMode) {
        if(rxmsg.DLC < 1) {
          USBSendError(rxDeviceId,CET_UnexpectedPacketSize,CPT_ReportParam,rxmsg.DLC);
          break;
        }
        struct PacketParam8ByteC reply;
        reply.m_header.m_packetType = CPT_ReportParam;
        reply.m_header.m_deviceId = rxDeviceId;
        reply.m_header.m_index = rxmsg.data8[0];
        int dlen = rxmsg.DLC-1;
        memcpy(reply.m_data.uint8,&rxmsg.data8[1],dlen);
        USBSendPacket((uint8_t *) &reply,sizeof(reply.m_header) + dlen);
      }
    } break;
    case CPT_AnnounceId: {
      if(g_canBridgeMode) {
        if(rxmsg.DLC != 8) {
          USBSendError(rxDeviceId,CET_UnexpectedPacketSize,CPT_ReportParam,rxmsg.DLC);
          break;
        }
        struct PacketDeviceIdC pkt;
        pkt.m_packetType = CPT_AnnounceId;
        pkt.m_deviceId = rxDeviceId;
        pkt.m_uid[0] = rxmsg.data32[0];
        pkt.m_uid[1] = rxmsg.data32[1];
        USBSendPacket((uint8_t *) &pkt,sizeof(struct PacketDeviceIdC));
      }
      break;
    }
    case CPT_ReadParam: {
      if(rxDeviceId == g_deviceId || rxDeviceId == 0) {
        if(rxmsg.DLC != 1) {
          CANSendError(CET_UnexpectedPacketSize,CPT_ReadParam,rxmsg.DLC);
          break;
        }
        enum ComsParameterIndexT index = (enum ComsParameterIndexT) rxmsg.data8[0];
        CANSendParam(index);
      }
    } break;
    case CPT_SetParam: {
      if(rxDeviceId == g_deviceId || rxDeviceId == 0) {
        if(rxmsg.DLC < 1) {
          CANSendError(CET_UnexpectedPacketSize,CPT_SetParam,rxmsg.DLC);
          break;
        }
        enum ComsParameterIndexT index = (enum ComsParameterIndexT) rxmsg.data8[0];
        union BufferTypeT dataBuff;
        int len = ((int)rxmsg.DLC)-1;
        if(len > 7) len = 7;
        memcpy(dataBuff.uint8,&rxmsg.data8[1],len);
        if(!SetParam(index,&dataBuff,len)) {
          CANSendError(CET_ParameterOutOfRange,CPT_SetParam,index);
        }
      }
#if 0
      if(g_canBridgeMode) {
        // Forward SetParam to bridge, useful for debugging.
        struct PacketParam8ByteC reply;
        reply.m_header.m_packetType = CPT_SetParam;
        reply.m_header.m_deviceId = rxDeviceId;
        reply.m_header.m_index = rxmsg.data8[0];
        int dlen = rxmsg.DLC-1;
        memcpy(reply.m_data.uint8,&rxmsg.data8[1],dlen);
        USBSendPacket((uint8_t *) &reply,sizeof(reply.m_header) + dlen);
      }
#endif

    } break;
    case CPT_SetDeviceId: {
      if(rxmsg.DLC != 8) {
        CANSendError(CET_UnexpectedPacketSize,CPT_SetDeviceId,rxmsg.DLC);
        break;
      }
      // Check if we're the targeted node.
      if(rxmsg.data32[0] != g_nodeUId[0] ||
          rxmsg.data32[1] != g_nodeUId[1]) {
        break;
      }
      g_deviceId = rxDeviceId;
      CANSendAnnounceId();
    } break;
    case CPT_QueryDevices: {
      if(rxmsg.DLC != 0) {
        CANSendError(CET_UnexpectedPacketSize,CPT_QueryDevices,rxmsg.DLC);
        break;
      }
      CANSendAnnounceId();
    } break;
    case CPT_ServoReport:
      if(rxDeviceId == g_otherJointId &&
          g_otherJointId != 0 &&
          g_otherJointId != g_deviceId) {
        MotionOtherJointUpdate(rxmsg.data16[0],rxmsg.data16[1],rxmsg.data8[4]);
      }
      if(g_canBridgeMode) {
        if(rxmsg.DLC != 5) {
          USBSendError(rxDeviceId,CET_UnexpectedPacketSize,CPT_ServoReport,rxmsg.DLC);
          break;
        }
        struct PacketServoReportC pkt;
        pkt.m_packetType = CPT_ServoReport;
        pkt.m_deviceId = rxDeviceId;
        pkt.m_position = rxmsg.data16[0];
        pkt.m_torque = rxmsg.data16[1];
        pkt.m_mode = rxmsg.data8[4];
        USBSendPacket((uint8_t *) &pkt,sizeof(struct PacketServoReportC));
      }
      break;
    case CPT_Servo: {
      if(rxDeviceId == g_deviceId && rxDeviceId != 0) {
        if(rxmsg.DLC != 5) {
          CANSendError(CET_UnexpectedPacketSize,CPT_Servo,rxmsg.DLC);
          break;
        }
        uint16_t position = rxmsg.data16[0];
        uint16_t torque = rxmsg.data16[1];
        uint8_t mode = rxmsg.data8[4];
        MotionSetPosition(mode,position,torque);
      }
    } break;
    case CPT_SaveSetup: {
      if(rxDeviceId == g_deviceId || rxDeviceId == 0) {
        enum FaultCodeT ret = SaveSetup();
        if(ret != FC_Ok) {
          CANSendError(CET_InternalError,CPT_SaveSetup,ret);
        }
      }
    } break;
    case CPT_LoadSetup: {
      if(rxDeviceId == g_deviceId || rxDeviceId == 0) {
        enum FaultCodeT ret = LoadSetup();
        if(ret != FC_Ok) {
          CANSendError(CET_InternalError,CPT_LoadSetup,ret);
        }
      }
    } break;
    case CPT_CalZero: {
      if(rxDeviceId == g_deviceId || rxDeviceId == 0) {
        if(!MotionCalZero()) {
          CANSendError(CET_MotorNotRunning,CPT_CalZero,0);
        }

      }
    } break;
    case CPT_BridgeMode: {
      // Shouldn't see this on the CAN bus, so report an error and drop it.
      CANSendError(CET_InternalError,CPT_BridgeMode,0);
    } break;
    case CPT_SyncTime: {
      // Not implemented yet.
      CANSendError(CET_NotImplemented,CPT_SyncTime,0);
    } break;
    default: {
      CANSendError(CET_NotImplemented,msgType,0);
    } break;
  }

  return true;
}

