
#include "canbus.h"
#include "can_coms.hh"
#include "can_queue.hh"
#include <string.h>
#include "bootloader.h"
#include "flashops.hh"

// Code for processing incoming CAN packets.

bool CANRecieveFrame(CANRxFrame *rxmsgptr)
{
  CANRxFrame &rxmsg = *rxmsgptr;


  int rxDeviceId = rxmsg.SID  & CAN_MSG_NODE_MASK;
  int msgType =  (rxmsg.SID >> CAN_MSG_TYPEBIT) & CAN_MSG_TYPEMASK;

  switch((enum ComsPacketTypeT) msgType)
  {
    case CPT_Ping: { // Ping request
      if(g_deviceId == rxDeviceId || rxDeviceId == 0) {
        if(rxmsg.DLC != 0) {
          CANSendError(CET_UnexpectedPacketSize,CPT_Ping,rxmsg.DLC);
          break;
        }
        CANPing(CPT_Pong,g_deviceId);
      }
    } break;
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
        if(SetParam(index,&dataBuff,len)) {
          SendParamUpdate(index);
        } else {
          CANSendError(CET_ParameterOutOfRange,CPT_SetParam,index);
        }
      }

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
    case CPT_BridgeMode: {
      // Shouldn't see this on the CAN bus, so report an error and drop it.
      CANSendError(CET_InternalError,CPT_BridgeMode,0);
    } break;

    case CPT_FlashCmdResult: // Status from a flash command, ignore
      break;
    case CPT_FlashEraseSector: {// Erase a flash sector

      if(rxDeviceId == g_deviceId || rxDeviceId == 0) {
        if(rxmsg.DLC != 5) {
          CANSendError(CET_UnexpectedPacketSize,msgType,rxmsg.DLC);
          break;
        }
        BootLoaderErase(rxmsg.data8[4],rxmsg.data32[0]);
      }
    } break;
    case CPT_FlashCmdReset: {// Erase a flash sector
      if(rxDeviceId == g_deviceId || rxDeviceId == 0) {
        if(rxmsg.DLC != 1) {
          CANSendError(CET_UnexpectedPacketSize,msgType,rxmsg.DLC);
          break;
        }
        BootLoaderReset(rxmsg.data8[0] != 0);
      }
    } break;
    case CPT_FlashChecksum: { // Generate a checksum
      if(rxDeviceId == g_deviceId || rxDeviceId == 0) {
        if(rxmsg.DLC != 7) {
          CANSendError(CET_UnexpectedPacketSize,msgType,rxmsg.DLC);
          break;
        }
        BootLoaderCheckSum(rxmsg.data8[6],rxmsg.data32[0],rxmsg.data16[2]);
      }
    } break;
    case CPT_FlashData: {     // Data packet
      if(rxDeviceId == g_deviceId || rxDeviceId == 0) {
        if(rxmsg.DLC == 0) {
          CANSendError(CET_UnexpectedPacketSize,msgType,rxmsg.DLC);
          break;
        }
        BootLoaderData(rxmsg.data8[0],&rxmsg.data8[1],rxmsg.DLC-1);
      }
    } break;
    case CPT_FlashWrite: {    // Write buffer
      if(rxDeviceId == g_deviceId || rxDeviceId == 0) {
        if(rxmsg.DLC != 7) {
          CANSendError(CET_UnexpectedPacketSize,msgType,rxmsg.DLC);
          break;
        }
        BootLoaderBeginWrite(rxmsg.data8[6],rxmsg.data32[0],rxmsg.data16[2]);
      }
    } break;
    case CPT_FlashRead:  {    // Read buffer and send it back
      if(rxDeviceId == g_deviceId || rxDeviceId == 0) {
        if(rxmsg.DLC != 7) {
          CANSendError(CET_UnexpectedPacketSize,msgType,rxmsg.DLC);
          break;
        }
        BootLoaderBeginRead(rxmsg.data8[6],rxmsg.data32[0],rxmsg.data16[2]);
      }
    } break;
    case CPT_NoOp:
    case CPT_EmergencyStop:
    case CPT_Pong:
    case CPT_Error:
    case CPT_Sync:
    case CPT_PWMState:
    case CPT_ReportParam:
    case CPT_AnnounceId:
      // Just drop these.
      break;
    case CPT_SyncTime:
    case CPT_ServoReport:
    case CPT_Servo:
    case CPT_SaveSetup:
    case CPT_LoadSetup:
    case CPT_CalZero:
      if(g_deviceId == rxDeviceId || rxDeviceId == 0)
        CANSendError(CET_NotImplemented,msgType,0);
      break;
    default: {
      if(g_deviceId == rxDeviceId)
        CANSendError(CET_NotImplemented,msgType,0);
    } break;
  }

  return true;
}

