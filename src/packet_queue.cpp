
#include "packet_queue.hh"
#include "bmc.h"
#include <string.h>

PacketQueueC::PacketQueueC()
{
  chMBObjectInit(&m_emptyPackets,m_emptyPacketData,PACKET_QUEUE_SIZE);
  for(int i = 0;i < PACKET_QUEUE_SIZE;i++) {
    chMBPost(&m_emptyPackets,reinterpret_cast<msg_t>(&m_packetArray[i]),TIME_IMMEDIATE);
  }
  chMBObjectInit(&m_fullPackets,m_fullPacketData,PACKET_QUEUE_SIZE);
}

void PacketQueueC::ReturnEmptyPacketI(struct PacketT *pkt) {
  if(chMBPostI(&m_emptyPackets,(msg_t) pkt) != MSG_OK)
    g_usbErrorCount++;
}

// Fetch a full packet.
struct PacketT *PacketQueueC::FetchFullI() {
  msg_t txMsg;
  if(chMBFetchI(&m_fullPackets,&txMsg) == MSG_OK) {
    struct PacketT *packet = reinterpret_cast<struct PacketT *>(txMsg);
    return packet;
  }
  return 0;
}

// Fetch a full packet.
struct PacketT *PacketQueueC::FetchFull(systime_t timeout) {
  msg_t txMsg;
  if(chMBFetch(&m_fullPackets,&txMsg,timeout) == MSG_OK) {
    struct PacketT *packet = reinterpret_cast<struct PacketT *>(txMsg);
    return packet;
  }
  return 0;
}


struct PacketT *PacketQueueC::GetEmptyPacket(systime_t timeout)
{
  msg_t msg;
  if(chMBFetch(&m_emptyPackets,&msg,timeout) != MSG_OK)
    return 0;
  return (PacketT *)msg;
}

struct PacketT *PacketQueueC::GetEmptyPacketI()
{
  msg_t msg;
  if(chMBFetchI(&m_emptyPackets,&msg) != MSG_OK)
    return 0;
  return (PacketT *)msg;
}


bool PacketQueueC::PostFullPacket(struct PacketT *pkt)
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
bool PacketQueueC::PostFullPacketI(struct PacketT *pkt)
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

//! Send a packet
bool PacketQueueC::SendPacket(uint8_t *buff,int len)
{
  struct PacketT *pkt;
  // If packet is too large, drop it log and flag an error occurred.
  if(len >= (int)sizeof(pkt->m_data) || len < 0) {
    g_usbErrorCount++;
    return false;
  }

  if((pkt = GetEmptyPacket(TIME_IMMEDIATE)) == 0) {
    g_usbDropCount++;
    return false;
  }

  pkt->m_len = len;
  memcpy(&pkt->m_data,buff,len);
  return PostFullPacket(pkt);
}

// ============================================================

// Common packet transmit code.

PacketQueueC g_txPacketQueue;
PacketQueueC g_txIntrPacketQueue;

/* Get a free packet structure. */
struct PacketT *USBGetEmptyPacket(systime_t timeout)
{
  if(!g_comsInitDone)
    return 0;
  return g_txPacketQueue.GetEmptyPacket(timeout);
}

/* Post full packet. */
bool USBPostPacket(struct PacketT *pkt)
{
  return g_txPacketQueue.PostFullPacket(pkt);
}

bool USBSendPacket(
    uint8_t *buff,
    int len
    )
{
  if(len <= 0)
    return false;
#if USE_PACKETUSB && BMC_USE_USB_EXTRA_ENDPOINTS
  // Peek at the packet type to decide how to handle it.
  switch((enum ComsPacketTypeT)buff[0]) {
    case CPT_PWMState:      // PWM State. Packet holding internal controller data.
    case CPT_Servo:         // Servo control position
    case CPT_ServoReport:   // Report servo position
      return g_txPacketQueue.SendPacket(buff,len);

    default:
      return g_txIntrPacketQueue.SendPacket(buff,len);
  }
  return false;
#else
  return g_txPacketQueue.SendPacket(buff,len);
#endif
}

