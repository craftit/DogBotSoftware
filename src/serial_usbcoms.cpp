

#include "serial_packet.hh"
#include "coms.h"
#include "canbus.h"
#include "dogbot/protocol.h"
#include "chmboxes.h"
#include "pwm.h"
#include "mathfunc.h"
#include "motion.h"
#include "drv8503.h"
#include "hal_channels.h"

#include <string.h>


#define PACKET_QUEUE_SIZE 8

static msg_t g_emptyPacketData[PACKET_QUEUE_SIZE];
static mailbox_t g_emptyPackets;
static msg_t g_fullPacketData[PACKET_QUEUE_SIZE];
static mailbox_t g_fullPackets;

static PacketT g_packetArray[PACKET_QUEUE_SIZE];



static THD_WORKING_AREA(waThreadRxComs, 512);
static THD_FUNCTION(ThreadRxComs, arg) {

  (void)arg;
  chRegSetThreadName("rxcoms");
  const static int buffSize = 256;
  static uint8_t buff[buffSize];
  /* Registering on the serial driver as event 1, interested in
       error flags and data-available only, other flags will not wakeup
       the thread.*/
  event_listener_t serial_listener;
  chEvtRegisterMaskWithFlags(chnGetEventSource(g_comsDecode.m_SDU),
                             &serial_listener,
                             EVENT_MASK(0),
                             CHN_INPUT_AVAILABLE | CHN_CONNECTED | CHN_DISCONNECTED
                             );
  while(true) {
    /* Waiting for any of the events we're registered on.*/
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,100);
    if (evt & EVENT_MASK(0)) {
      // Event from the serial interface, getting serial flags as first thing.
      eventflags_t flags = chEvtGetAndClearFlags(&serial_listener);

      // Got a new connection ?
      if (flags & (CHN_CONNECTED)) {
        g_comsDecode.ResetStateMachine();
      }
      // Disable bridge mode on disconnect.
      if (flags & (CHN_DISCONNECTED)) {
        g_canBridgeMode = false;
      }
      if (flags & CHN_INPUT_AVAILABLE) {
        // Try and read a block of data, but with zero timeout.
        while(true) {
          int ret = chnReadTimeout(g_comsDecode.m_SDU,buff,buffSize,TIME_IMMEDIATE);
          if(ret <= 0)
            break;
          for(int i = 0;i < ret;i++) {
            g_comsDecode.AcceptByte(buff[i]);
          }
        }
      }
    }
  }
}

static THD_WORKING_AREA(waThreadTxComs, 256);
static THD_FUNCTION(ThreadTxComs, arg) {

  (void)arg;
  chRegSetThreadName("txcoms");
  static uint8_t txbuff[70];
  while(true) {
    struct PacketT *packet = 0;
    if(chMBFetch(&g_fullPackets,(msg_t*) &packet,TIME_INFINITE) != MSG_OK) {
      // Prevent a possible tight loop
      chThdSleepMilliseconds(10);
      continue;
    }

    if(SDU1.config->usbp->state == USB_ACTIVE) {

      uint8_t len = packet->m_len;

      uint8_t *buff = packet->m_data;

      uint8_t *at = txbuff;
      *(at++) = g_charSTX;
      *(at++) = len;
      int crc = len + 0x55;
      for(int i = 0;i < len;i++) {
        uint8_t data = buff[i];
        *(at++) = data;
        crc += data;
      }
      *(at++) = crc;
      *(at++) = crc>>8;
      *(at++) = g_charETX;
      int size = at - txbuff;

      chnWrite(g_packetStream, txbuff, size);
    } else {
      // If we've lost our connection turn bridge mode off.
      g_canBridgeMode = false;
    }

    chMBPost(&g_emptyPackets,reinterpret_cast<msg_t>(packet),TIME_INFINITE);
  }
}


/* Get a free packet structure. */
struct PacketT *GetEmptyPacket(systime_t timeout)
{
  if(!g_comsInitDone)
    return 0;

  msg_t msg;
  if(chMBFetch(&g_emptyPackets,&msg,timeout) != MSG_OK)
    return 0;
  return (PacketT *)msg;
}



/* Post full packet. */
bool PostPacket(struct PacketT *pkt)
{
  if(chMBPost(&g_fullPackets,(msg_t) pkt,TIME_IMMEDIATE) == MSG_OK)
    return true;

  g_usbErrorCount++;
  FaultDetected(FC_InternalUSB);
  // This shouldn't happen, as if we can't acquire an empty buffer
  // unless there is space available, but we don't want to loose the buffer
  // so attempt to add it back to the free list
  if(chMBPost(&g_emptyPackets,(msg_t) pkt,TIME_IMMEDIATE) != MSG_OK) {
    // Things are really screwy. Panic ?
  }
  return false;
}

bool USBSendPacket(
    uint8_t *buff,
    int len
    )
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
  return PostPacket(pkt);
}


BaseAsynchronousChannel *g_packetStream = 0;

void InitComs()
{
  if(g_comsInitDone)
    return ;

  g_comsDecode.m_SDU = (BaseAsynchronousChannel *)&SDU1;
  g_packetStream = (BaseAsynchronousChannel *)&SDU1;

  chMBObjectInit(&g_emptyPackets,g_emptyPacketData,PACKET_QUEUE_SIZE);
  for(int i = 0;i < PACKET_QUEUE_SIZE;i++) {
    chMBPost(&g_emptyPackets,reinterpret_cast<msg_t>(&g_packetArray[i]),TIME_IMMEDIATE);
  }
  chMBObjectInit(&g_fullPackets,g_fullPacketData,PACKET_QUEUE_SIZE);

  g_comsInitDone = true;

  chThdCreateStatic(waThreadTxComs, sizeof(waThreadTxComs), NORMALPRIO, ThreadTxComs, NULL);
  chThdCreateStatic(waThreadRxComs, sizeof(waThreadRxComs), NORMALPRIO+1, ThreadRxComs, NULL); // Prefer dealing with incoming messages.

}

