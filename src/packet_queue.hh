#ifndef PACKET_QUEUE_HEADER
#define PACKET_QUEUE_HEADER 1

#include "hal.h"
#include "bmc.h"
#include "coms.h"

#define PACKET_QUEUE_SIZE 16

/* TX/RX Data buffer management. */

class PacketQueueC {

public:
  PacketQueueC();

  struct PacketT *GetEmptyPacket(systime_t timeout);

  /* Post full packet. */
  bool PostFullPacket(struct PacketT *pkt);

  /* Get an empty packet for filling. */
  struct PacketT *GetEmptyPacketI();

  /* Post full packet. */
  bool PostFullPacketI(struct PacketT *pkt);

  // Return an empty packet to the queue.
  void ReturnEmptyPacketI(struct PacketT *pkt);

  //! Send a packet
  bool SendPacket(uint8_t *buff,int len);

  // Fetch a full packet.
  struct PacketT *FetchFullI();

  // Fetch a full packet.
  struct PacketT *FetchFull(systime_t timeout);

  msg_t m_emptyPacketData[PACKET_QUEUE_SIZE];
  mailbox_t m_emptyPackets;

  msg_t m_fullPacketData[PACKET_QUEUE_SIZE];
  mailbox_t m_fullPackets;

  PacketT m_packetArray[PACKET_QUEUE_SIZE];
};

extern PacketQueueC g_txPacketQueue;

#endif
