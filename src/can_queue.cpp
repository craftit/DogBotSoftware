
#include "can_queue.hh"
#include "bmc.h"
#include <string.h>

CANQueueC::CANQueueC()
{
  chMBObjectInit(&m_emptyPackets,m_emptyPacketData,CAN_QUEUE_SIZE);
  for(int i = 0;i < CAN_QUEUE_SIZE;i++) {
    chMBPost(&m_emptyPackets,reinterpret_cast<msg_t>(&m_packetArray[i]),TIME_IMMEDIATE);
  }
  chMBObjectInit(&m_fullPackets,m_fullPacketData,CAN_QUEUE_SIZE);
}

void CANQueueC::ReturnEmptyPacketI(CANTxFrame *pkt) {
  if(chMBPostI(&m_emptyPackets,(msg_t) pkt) != MSG_OK)
    g_usbErrorCount++;
}

// Fetch a full packet.
CANTxFrame *CANQueueC::FetchFullI() {
  msg_t txMsg;
  if(chMBFetchI(&m_fullPackets,&txMsg) == MSG_OK) {
    CANTxFrame *packet = reinterpret_cast<CANTxFrame *>(txMsg);
    return packet;
  }
  return 0;
}

// Fetch a full packet.
CANTxFrame *CANQueueC::FetchFull(systime_t timeout) {
  msg_t txMsg;
  if(chMBFetch(&m_fullPackets,&txMsg,timeout) == MSG_OK) {
    CANTxFrame *packet = reinterpret_cast<CANTxFrame *>(txMsg);
    return packet;
  }
  return 0;
}


CANTxFrame *CANQueueC::GetEmptyPacket(systime_t timeout)
{
  msg_t msg;
  if(chMBFetch(&m_emptyPackets,&msg,timeout) != MSG_OK)
    return 0;
  return (CANTxFrame *)msg;
}

CANTxFrame *CANQueueC::GetEmptyPacketI()
{
  msg_t msg;
  if(chMBFetchI(&m_emptyPackets,&msg) != MSG_OK)
    return 0;
  return (CANTxFrame *)msg;
}


bool CANQueueC::PostFullPacket(CANTxFrame *pkt)
{
  if(chMBPost(&m_fullPackets,(msg_t) pkt,TIME_IMMEDIATE) == MSG_OK)
    return true;

  g_usbErrorCount++;
  FaultDetected(FC_InternalUSB);
  // This shouldn't happen, as if we can't acquire an empty buffer
  // unless there is space available, but we don't want to loose the buffer
  // so attempt to add it back to the free list
  if(chMBPost(&m_emptyPackets,(msg_t) pkt,TIME_IMMEDIATE) != MSG_OK) {
    // Things are really screwy. Panic ?
  }
  return false;
}

/* Post full packet. */
bool CANQueueC::PostFullPacketI(CANTxFrame *pkt)
{
  if(chMBPostI(&m_fullPackets,(msg_t) pkt) == MSG_OK)
    return true;

  g_usbErrorCount++;
  FaultDetected(FC_InternalUSB);
  // This shouldn't happen, as if we can't acquire an empty buffer
  // unless there is space available, but we don't want to loose the buffer
  // so attempt to add it back to the free list
  if(chMBPostI(&m_emptyPackets,(msg_t) pkt) != MSG_OK) {
    // Things are really screwy. Panic ?
  }
  return false;
}


// ============================================================

// Common packet transmit code.

CANQueueC g_txCANQueue;

