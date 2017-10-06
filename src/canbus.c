/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include <stdint.h>
#include <string.h>
#include "coms.h"
#include "canbus.h"
#include "dogbot/protocol.h"
#include "motion.h"

#define STM32_UID ((uint32_t *)0x1FFF7A10)

uint32_t g_nodeUId[2]; // Not absolutely guaranteed to be unique but collisions are unlikely;

uint8_t g_deviceId = 0;

#define MSG_TYPE_BIT 6
#define MSG_NODE_MASK 0x3f
#define MSG_TYPE_MASK 0x1f

bool CANSetAddress(CANTxFrame *txmsg,int nodeId,int packetType)
{
  txmsg->SID = (packetType & 0x1f) << MSG_TYPE_BIT | (nodeId & 0x3f);
  txmsg->IDE = CAN_IDE_STD;

  return true;
}

bool CANPing(
    enum ComsPacketTypeT pktType, // Must be either ping or pong
    uint8_t deviceId
    )
{
  if(pktType != CPT_Pong && pktType != CPT_Ping)
    return false;

  CANTxFrame txmsg;
  CANSetAddress(&txmsg,deviceId,pktType);
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 0;
  if(canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, 100) != MSG_OK) {
    //SendError(CET_CANTransmitFailed,m_data[0]);
    return false;
  }

  return true;
}

bool CANSendServoReport(
    uint8_t deviceId,
    uint16_t position,
    int16_t torque,
    uint8_t state
    )
{
  CANTxFrame txmsg;
  CANSetAddress(&txmsg,deviceId,CPT_ServoReport);
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 5;
  txmsg.data16[0] = position;
  txmsg.data16[1] = torque;
  txmsg.data8[4] = state;
  if(canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, 100) != MSG_OK) {
    //SendError(CET_CANTransmitFailed,m_data[0]);
    return false;
  }
  return true;
}

bool CANSendServo(
    uint8_t deviceId,
    uint16_t position,
    uint16_t torque,
    uint8_t state
    )
{
  CANTxFrame txmsg;
  CANSetAddress(&txmsg,deviceId,CPT_Servo);
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 5;
  txmsg.data16[0] = position;
  txmsg.data16[1] = torque;
  txmsg.data8[4] = state;
  if(canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, 100) != MSG_OK) {
    //SendError(CET_CANTransmitFailed,m_data[0]);
    return false;
  }
  return true;

}

bool CANSendQueryDevices(void)
{
  CANTxFrame txmsg;
  CANSetAddress(&txmsg,0,CPT_QueryDevices);
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 0;

  if(canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100)) != MSG_OK) {
    return false;
  }

  return true;
}

bool CANSendSetDevice(
    uint8_t deviceId,
    uint32_t uid0,
    uint32_t uid1
    )
{
  CANTxFrame txmsg;
  CANSetAddress(&txmsg,deviceId,CPT_SetDeviceId);
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 8;
  txmsg.data32[0] = uid0;
  txmsg.data32[1] = uid1;

  if(canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100)) != MSG_OK) {
    return false;
  }

  return true;
}


bool CANSendSetParam(
    uint8_t deviceId,
    uint16_t index,
    union BufferTypeT *data,
    int len
    )
{
  if(len > 7) {
    return false;
  }

  CANTxFrame txmsg;
  CANSetAddress(&txmsg,deviceId,CPT_SetParam);
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 1 + len;
  txmsg.data8[0] = index;
  memcpy(&txmsg.data8[1],data->uint8,len);
  if(canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100)) != MSG_OK) {
    return false;
  }

  return true;
}

bool CANSendReadParam(
    uint8_t deviceId,
    uint16_t index
    )
{
  CANTxFrame txmsg;
  CANSetAddress(&txmsg,deviceId,CPT_ReadParam);
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 1;
  txmsg.data8[0] = index;

  if(canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100)) != MSG_OK) {
    return false;
  }

  return true;
}

bool CANSendError(
    uint16_t errorCode,
    uint16_t data
    )
{
  CANTxFrame txmsg;
  CANSetAddress(&txmsg,g_deviceId,CPT_Error);
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 4;
  txmsg.data16[0] = errorCode;
  txmsg.data16[1] = data;

  if(canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100)) != MSG_OK) {
    return false;
  }

  return true;
}


bool CANSendParam(uint16_t index)
{
  union BufferTypeT buff;
  int len = -1;
  /* Retrieve the requested parameter information */
  if(!ReadParam((enum ComsParameterIndexT) index,&len,&buff)) {
    CANSendError(CET_ParameterOutOfRange, index);
    // Report error ?
    return false;
  }
  if(len <= 0) {
    CANSendError(CET_InternalError, index);
    // Report error ?
    return false;
  }

  CANTxFrame txmsg;
  CANSetAddress(&txmsg,g_deviceId,CPT_ReportParam);
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.data8[0] = index;

  // Limit length to bytes we can send over can.
  if(len > 7) len = 7;
  txmsg.DLC = len+1;
  memcpy(&txmsg.data8[1],buff.uint8,len);
  if(canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, 100) != MSG_OK) {
    //SendError(CET_CANTransmitFailed,m_data[0]);
    return false;
  }
  return true;
}





/*
 * Internal loopback mode, 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */
static const CANConfig cancfg = {
  CAN_MCR_ABOM |
  CAN_MCR_AWUM |
  CAN_MCR_TXFP,
//  CAN_BTR_LBKM |
  CAN_BTR_SJW(0) |
  CAN_BTR_TS2(1) |
  CAN_BTR_TS1(8) |
  CAN_BTR_BRP(6)
};

/*
 * Receiver thread.
 */
static THD_WORKING_AREA(can_rx_wa, 256);
static THD_FUNCTION(can_rx, p) {
  event_listener_t el;
  CANRxFrame rxmsg;

  (void)p;
  chRegSetThreadName("can receiver");
  chEvtRegister(&CAND1.rxfull_event, &el, 0);
  while(!chThdShouldTerminateX()) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
      continue;
    while (canReceive(&CAND1, CAN_ANY_MAILBOX,
                      &rxmsg, TIME_IMMEDIATE) == MSG_OK) {

      /* Process message.*/

      //palTogglePad(GPIOC, GPIOC_PIN5);       /* Yellow led. */

      int rxDeviceId = rxmsg.SID  & MSG_NODE_MASK;
      int msgType =  (rxmsg.SID >> MSG_TYPE_BIT) & MSG_TYPE_MASK;


      switch((enum ComsPacketTypeT) msgType)
      {
        case CPT_Ping: { // Ping request
          if(g_deviceId == rxDeviceId || rxDeviceId == 0) {
            if(rxmsg.DLC != 0) {
              CANSendError(CET_UnexpectedPacketSize,CPT_Ping);
              break;
            }
            CANPing(CPT_Pong,g_deviceId);
          }
        } break;
        case CPT_Pong: {
          if(g_canBridgeMode) {
            if(rxmsg.DLC != 0) {
              SendError(rxDeviceId,CET_UnexpectedPacketSize,CPT_Pong);
              break;
            }
            struct PacketPingPongC pkt;
            pkt.m_packetType = CPT_Pong;
            pkt.m_deviceId = rxDeviceId;
            SendPacket((uint8_t *) &pkt,sizeof(struct PacketPingPongC));
          }
        } break;
        case CPT_Error:
          if(g_canBridgeMode) {
            if(rxmsg.DLC != 4) {
              SendError(rxDeviceId,CET_UnexpectedPacketSize,CPT_Error);
              break;
            }
            struct PacketErrorC pkt;
            pkt.m_packetType = CPT_Error;
            pkt.m_deviceId = rxDeviceId;
            pkt.m_errorCode = rxmsg.data16[0];
            pkt.m_errorData = rxmsg.data16[1];
            SendPacket((uint8_t *) &pkt,sizeof(struct PacketErrorC));
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
              SendError(rxDeviceId,CET_UnexpectedPacketSize,CPT_ReportParam);
              break;
            }
            struct PacketParam8ByteC reply;
            reply.m_header.m_packetType = CPT_ReportParam;
            reply.m_header.m_deviceId = rxDeviceId;
            reply.m_header.m_index = rxmsg.data8[0];
            int dlen = rxmsg.DLC-1;
            memcpy(reply.m_data.uint8,&rxmsg.data8[1],dlen);
            SendPacket((uint8_t *) &reply,sizeof(reply.m_header) + dlen);
          }
        } break;
        case CPT_AnnounceId: {
          if(g_canBridgeMode) {
            if(rxmsg.DLC != 8) {
              SendError(rxDeviceId,CET_UnexpectedPacketSize,CPT_ReportParam);
              break;
            }
            struct PacketDeviceIdC pkt;
            pkt.m_packetType = CPT_AnnounceId;
            pkt.m_deviceId = rxDeviceId;
            pkt.m_uid[0] = rxmsg.data32[0];
            pkt.m_uid[1] = rxmsg.data32[1];
            SendPacket((uint8_t *) &pkt,sizeof(struct PacketDeviceIdC));
          }
          break;
        }
        case CPT_ReadParam: {
          if(rxDeviceId == g_deviceId || rxDeviceId == 0) {
            if(rxmsg.DLC != 1) {
              CANSendError(CET_UnexpectedPacketSize,CPT_ReadParam);
              break;
            }
            int index = rxmsg.data8[0];
            CANSendParam(index);
          }
        } break;
        case CPT_SetParam: {
          if(rxDeviceId == g_deviceId || rxDeviceId == 0) {
            if(rxmsg.DLC < 1) {
              CANSendError(CET_UnexpectedPacketSize,CPT_SetParam);
              break;
            }
            int index = rxmsg.data8[0];
            union BufferTypeT dataBuff;
            int len = rxmsg.DLC-1;
            if(len > 7) len = 7;
            memcpy(dataBuff.uint8,&rxmsg.data8[1],len);
            if(!SetParam((enum ComsParameterIndexT) index,&dataBuff,len)) {
              CANSendError(CET_ParameterOutOfRange,index);
            }
          }

        } break;
        case CPT_SetDeviceId: {
          if(rxmsg.DLC != 8) {
            CANSendError(CET_UnexpectedPacketSize,CPT_SetDeviceId);
            break;
          }
          // Check if we're the targeted node.
          if(rxmsg.data32[0] != g_nodeUId[0] ||
              rxmsg.data32[1] != g_nodeUId[1]) {
            break;
          }
          g_deviceId = rxDeviceId;
        }
        /* Fall through and announce new id. */
        /* no break */
        case CPT_QueryDevices: {
          if(rxmsg.DLC != 0) {
            CANSendError(CET_UnexpectedPacketSize,CPT_QueryDevices);
            break;
          }
          CANTxFrame txmsg;
          CANSetAddress(&txmsg,g_deviceId,CPT_AnnounceId);
          txmsg.RTR = CAN_RTR_DATA;
          txmsg.DLC = 8;
          txmsg.data32[0] = g_nodeUId[0];
          txmsg.data32[1] = g_nodeUId[1];
          if(canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, 100) != MSG_OK) {
            if(g_canBridgeMode) {
              SendError(rxDeviceId,CET_CANTransmitFailed,CPT_QueryDevices);
            }
          }
        } break;
        case CPT_ServoReport:
          if(g_canBridgeMode) {
            if(rxmsg.DLC != 5) {
              SendError(rxDeviceId,CET_UnexpectedPacketSize,CPT_ServoReport);
              break;
            }
            struct PacketServoReportC pkt;
            pkt.m_packetType = CPT_ServoReport;
            pkt.m_deviceId = rxDeviceId;
            pkt.m_position = rxmsg.data16[0];
            pkt.m_torque = rxmsg.data16[1];
            pkt.m_mode = rxmsg.data8[4];
            SendPacket((uint8_t *) &pkt,sizeof(struct PacketServoReportC));
          }
          break;
        case CPT_Servo: {
          if(rxDeviceId == g_deviceId && rxDeviceId != 0) {
            if(rxmsg.DLC != 5) {
              SendError(rxDeviceId,CET_UnexpectedPacketSize,CPT_Servo);
              break;
            }
            uint16_t position = rxmsg.data16[0];
            uint16_t torque = rxmsg.data16[1];
            uint8_t mode = rxmsg.data8[4];
            MotionSetPosition(mode,position,torque);
          }
        } break;
      }

    }
  }
  chEvtUnregister(&CAND1.rxfull_event, &el);
}

#if 0
/*
 * Transmitter thread.
 */
static THD_WORKING_AREA(can_tx_wa, 256);
static THD_FUNCTION(can_tx, p) {
  CANTxFrame txmsg;

  (void)p;
  chRegSetThreadName("can transmitter");
  txmsg.IDE = CAN_IDE_EXT;
  txmsg.EID = 0x01234567;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 8;
  txmsg.data32[0] = 0x55AA55AA;
  txmsg.data32[1] = 0x00FF00FF;

  while (!chThdShouldTerminateX()) {
    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
    chThdSleepMilliseconds(500);
  }
}
#endif

/*
 * Application entry point.
 */
int InitCAN(void)
{
  // Setup an unique node id.
  g_nodeUId[0] = STM32_UID[0];
  g_nodeUId[1] = STM32_UID[1] + STM32_UID[2];

  palClearPad(GPIOB, GPIOB_PIN5);       /* Make sure transmitter is in normal mode.  */

  static bool canInitDone = false;
  if(!canInitDone) {
    canInitDone = true;
    /*
     * Activates the CAN driver 1.
     */
    canStart(&CAND1, &cancfg);

    /*
     * Starting the transmitter and receiver threads.
     */
    chThdCreateStatic(can_rx_wa, sizeof(can_rx_wa), NORMALPRIO + 1,
                      can_rx, NULL);
  #if 0
    chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO + 7,
                      can_tx, NULL);
  #endif
  }
  return 0;
}
