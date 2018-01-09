#ifndef PACKET_QUEUE_HEADER
#define PACKET_QUEUE_HEADER 1

#include "hal.h"

#define CAN_QUEUE_SIZE 24

/* TX/RX Data buffer management. */

class CANQueueC {

public:
  CANQueueC();

  CANTxFrame *GetEmptyPacket(systime_t timeout);

  /* Post full packet. */
  bool PostFullPacket(CANTxFrame *pkt);

  /* Get an empty packet for filling. */
  CANTxFrame *GetEmptyPacketI();

  /* Post full packet. */
  bool PostFullPacketI(CANTxFrame *pkt);

  // Return an empty packet to the queue.
  void ReturnEmptyPacketI(CANTxFrame *pkt);

  // Fetch a full packet.
  CANTxFrame *FetchFullI();

  // Fetch a full packet.
  CANTxFrame *FetchFull(systime_t timeout);

  msg_t m_emptyPacketData[CAN_QUEUE_SIZE];
  mailbox_t m_emptyPackets;

  msg_t m_fullPacketData[CAN_QUEUE_SIZE];
  mailbox_t m_fullPackets;

  CANTxFrame m_packetArray[CAN_QUEUE_SIZE];

protected:
  void Init();
};

extern int g_canErrorCount;
extern int g_canDropCount;
extern CANQueueC g_txCANQueue;

#endif
