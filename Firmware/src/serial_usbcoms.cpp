
#include "serial_packet.hh"
#include "bmc.h"
#include "packet_queue.hh"
#include "coms.h"
#include "canbus.h"
#include "dogbot/protocol.h"
#include "chmboxes.h"
#include "pwm.h"
#include "mathfunc.h"
#include "motion.h"
#include "hal_channels.h"

#include <string.h>
#include "drv8305.h"

#if !USE_PACKETUSB

BaseAsynchronousChannel *g_packetStream = 0;

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

    msg_t txMsg;
    struct PacketT *packet = g_txPacketQueue.FetchFull(txMsg,TIME_INFINITE);

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

    g_txPacketQueue.ReturnEmptyPacketI(txMsg);
  }
}



void InitComs()
{
  if(g_comsInitDone)
    return ;

  g_comsDecode.m_SDU = (BaseAsynchronousChannel *)&SDU1;
  g_packetStream = (BaseAsynchronousChannel *)&SDU1;

  g_comsInitDone = true;

  chThdCreateStatic(waThreadTxComs, sizeof(waThreadTxComs), NORMALPRIO, ThreadTxComs, NULL);
  chThdCreateStatic(waThreadRxComs, sizeof(waThreadRxComs), NORMALPRIO+1, ThreadRxComs, NULL); // Prefer dealing with incoming messages.

}

#endif
