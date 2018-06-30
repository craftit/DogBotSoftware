

#include "ch.h"
#include "hal.h"
#include <stdint.h>
#include <string.h>
#include "canbus.h"
#include "can_queue.hh"
#include "can_coms.hh"
#include "coms.h"

#define STM32_UID ((uint32_t *)0x1FFF7A10)

uint32_t g_nodeUId[2]; // Not absolutely guaranteed to be unique but collisions are very unlikely;
uint8_t g_deviceId = 0;

bool CANSetAddress(CANTxFrame *txmsg,int nodeId,int packetType)
{
  txmsg->SID = (packetType & 0x1f) << CAN_MSG_TYPEBIT | (nodeId & 0x3f);
  txmsg->IDE = CAN_IDE_STD;

  return true;
}

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
    int16_t velocity,
    int16_t torque,
    uint8_t state,
    uint8_t timeStamp
    )
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,CPT_ServoReport);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 8;
  txmsg->data16[0] = position;
  txmsg->data16[1] = torque;
  txmsg->data8[4] = state;
  txmsg->data8[5] = timeStamp;
  txmsg->data16[3] = velocity;
  return g_txCANQueue.PostFullPacket(txmsg);
}

bool CANSendServo(
    uint8_t deviceId,
    int16_t position,
    int16_t torque,
    uint8_t state,
    uint8_t timestamp
    )
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;

  CANSetAddress(txmsg,deviceId,CPT_Servo);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 5;
  txmsg->data16[0] = position;
  txmsg->data16[1] = torque;
  txmsg->data8[4] = state;
  txmsg->data8[5] = timestamp;
  return g_txCANQueue.PostFullPacket(txmsg);
}

// Send an emergency stop
bool CANEmergencyStop(uint8_t deviceId,enum StateChangeSourceT eStopCause)
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;

  enum ComsPacketTypeT pktType = CPT_EmergencyStop;
  CANSetAddress(txmsg,0,pktType);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 2;
  txmsg->data8[0] = deviceId;
  txmsg->data8[1] = eStopCause;
  return g_txCANQueue.PostFullPacket(txmsg);
}

bool CANSyncTime()
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  enum ComsPacketTypeT pktType = CPT_SyncTime;
  CANSetAddress(txmsg,0,pktType);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 0;
  return g_txCANQueue.PostFullPacket(txmsg);
}

bool CANPing(
    enum ComsPacketTypeT pktType, // Must be either ping or pong
    uint8_t deviceId,
    uint16_t payload
    )
{
  if(pktType != CPT_Pong && pktType != CPT_Ping)
    return false;

  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,pktType);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 2;
  txmsg->data16[0] = payload;
  return g_txCANQueue.PostFullPacket(txmsg);
}


bool CANSendQueryDevices(void)
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,0,CPT_QueryDevices);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 0;
  return g_txCANQueue.PostFullPacket(txmsg);
}

bool CANSendSetDevice(
    uint8_t deviceId,
    uint32_t uid0,
    uint32_t uid1
    )
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,CPT_SetDeviceId);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 8;
  txmsg->data32[0] = uid0;
  txmsg->data32[1] = uid1;
  return g_txCANQueue.PostFullPacket(txmsg);
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

  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,CPT_SetParam);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 1 + len;
  txmsg->data8[0] = index;
  memcpy(&txmsg->data8[1],data->uint8,len);
  return g_txCANQueue.PostFullPacket(txmsg);
}

bool CANSendReadParam(
    uint8_t deviceId,
    uint16_t index
    )
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,CPT_ReadParam);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 1;
  txmsg->data8[0] = index;
  return g_txCANQueue.PostFullPacket(txmsg);
}

bool CANSendError(
    uint16_t errorCode,
    uint8_t causeType,
    uint8_t data
    )
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,g_deviceId,CPT_Error);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 3;
  txmsg->data8[0] = errorCode;
  txmsg->data8[1] = causeType;
  txmsg->data8[2] = data;
  return g_txCANQueue.PostFullPacket(txmsg);
}


bool CANSendParam(enum ComsParameterIndexT index)
{
  union BufferTypeT buff;
  int len = -1;
  /* Retrieve the requested parameter information */
  if(!ReadParam(index,&len,&buff)) {
    CANSendError(CET_ParameterOutOfRange, CPT_ReadParam,(uint8_t) index);
    // Report error ?
    return false;
  }
  if(len <= 0 || len > 7) {
    CANSendError(CET_InternalError, CPT_ReadParam,(uint8_t) index);
    // Report error ?
    return false;
  }

  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0) // Queue is full ?
    return false;
  CANSetAddress(txmsg,g_deviceId,CPT_ReportParam);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->data8[0] = (uint8_t) index;

  txmsg->DLC = len+1;
  memcpy(&txmsg->data8[1],buff.uint8,len);
  return g_txCANQueue.PostFullPacket(txmsg);
}

bool CANSendAnnounceId()
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,g_deviceId,CPT_AnnounceId);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 8;
  txmsg->data32[0] = g_nodeUId[0];
  txmsg->data32[1] = g_nodeUId[1];
  g_txCANQueue.PostFullPacket(txmsg);
  return true;
}

bool CANSendBootLoaderReset(uint8_t deviceId,bool enable)
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,CPT_FlashCmdReset);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 1;
  txmsg->data8[0] = enable;
  g_txCANQueue.PostFullPacket(txmsg);
  return true;
}


bool CANSendBootLoaderResult(uint8_t deviceId,uint8_t lastSeqNum,enum BootLoaderStateT state,enum FlashOperationStatusT result)
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,CPT_FlashCmdResult);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 3;
  txmsg->data8[0] = lastSeqNum;
  txmsg->data8[1] = state;
  txmsg->data8[2] = result;
  g_txCANQueue.PostFullPacket(txmsg);
  return true;
}

bool CANSendBootLoaderErase(uint8_t deviceId,uint8_t seqNum,uint32_t blockAddr)
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,CPT_FlashEraseSector);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 5;
  txmsg->data32[0] = blockAddr;
  txmsg->data8[4] = seqNum;
  g_txCANQueue.PostFullPacket(txmsg);
  return true;
}

bool CANSendBootLoaderData(uint8_t deviceId,uint8_t seqNum,uint8_t *data,uint8_t len)
{
  if(len > 7)
    return false;
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,CPT_FlashData);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 1 + len;
  txmsg->data8[0] = seqNum;
  memcpy(&txmsg->data8[1],data,len);

  g_txCANQueue.PostFullPacket(txmsg);
  return true;
}

bool CANSendBootLoaderRead(uint8_t deviceId,uint8_t seqNum,uint32_t addr,uint16_t len)
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,CPT_FlashRead);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 7;
  txmsg->data32[0] = addr;
  txmsg->data16[2] = len;
  txmsg->data8[6] = seqNum;
  g_txCANQueue.PostFullPacket(txmsg);
  return true;
}

bool CANSendBootLoaderWrite(uint8_t deviceId,uint8_t seqNum,uint32_t addr,uint16_t len)
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,CPT_FlashWrite);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 7;
  txmsg->data32[0] = addr;
  txmsg->data16[2] = len;
  txmsg->data8[6] = seqNum;
  g_txCANQueue.PostFullPacket(txmsg);
  return true;
}

bool CANSendBootLoaderCheckSum(uint8_t deviceId,uint8_t seqNum,uint32_t addr,uint16_t len)
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,CPT_FlashChecksum);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 7;
  txmsg->data32[0] = addr;
  txmsg->data16[2] = len;
  txmsg->data8[6] = seqNum;
  g_txCANQueue.PostFullPacket(txmsg);
  return true;
}

bool CANSendBootLoaderCheckSumResult(uint8_t deviceId,uint8_t seqNum,uint32_t sum)
{
  CANTxFrame *txmsg = g_txCANQueue.GetEmptyPacketI();
  if(txmsg == 0)
    return false;
  CANSetAddress(txmsg,deviceId,CPT_FlashChecksumResult);
  txmsg->RTR = CAN_RTR_DATA;
  txmsg->DLC = 4;
  txmsg->data32[0] = sum;
  txmsg->data8[4] = seqNum;
  g_txCANQueue.PostFullPacket(txmsg);
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
  CAN_BTR_TS1(10) |
  CAN_BTR_BRP(2)
};

// For 500Kb
// CAN_BTR_SJW(0) |
// CAN_BTR_TS2(1) |
// CAN_BTR_TS1(8) |
// CAN_BTR_BRP(6)

// For 1Mb
// CAN_BTR_SJW(0) |
// CAN_BTR_TS2(1) |
// CAN_BTR_TS1(10) |
// CAN_BTR_BRP(2)


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
      CANRecieveFrame(&rxmsg);
    }
  }
  chEvtUnregister(&CAND1.rxfull_event, &el);
}

/*
 * Transmitter thread.
 */
static THD_WORKING_AREA(can_tx_wa, 128);
static THD_FUNCTION(can_tx, p)
{
  (void)p;
  chRegSetThreadName("can tx");

  while (!chThdShouldTerminateX()) {
    CANTxFrame *txPkt = g_txCANQueue.FetchFull(MS2ST(100));
    if(txPkt == 0)
      continue;
    if(canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, txPkt, MS2ST(50)) != MSG_OK) {
      g_canDropCount++;
    }
    g_txCANQueue.ReturnEmptyPacketI(txPkt);
  }
}

/*
 * CAN startup code
 */
int InitCAN(void)
{
  // Setup an unique node id.
  g_nodeUId[0] = (STM32_UID[0] & 0x0ffffff); // Upper 4 bits are reserved for device type
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
    chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO + 7,
                      can_tx, NULL);
  }

  CANSendAnnounceId();
  return 0;
}



