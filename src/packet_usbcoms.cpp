

#ifdef USE_PACKETUSB

#include "packet_usb.h"
#include "coms.h"
#include "hal.h"
#include "bmc.h"
#include <string.h>

bool g_packetUSBActive = false;

#define PACKET_QUEUE_SIZE 8

/* TX Data buffer management. */

static msg_t g_emptyPacketData[PACKET_QUEUE_SIZE];
static mailbox_t g_emptyPackets;

static msg_t g_fullPacketData[PACKET_QUEUE_SIZE];
static mailbox_t g_fullPackets;

static PacketT g_packetArray[PACKET_QUEUE_SIZE];



static THD_WORKING_AREA(waThreadRxComs, 512);
static THD_FUNCTION(ThreadRxComs, arg) {

  (void)arg;
  chRegSetThreadName("rxPacketComs");

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(1500);
  usbStart(&USBD1,&usbcfg);
  usbConnectBus(&USBD1);

  while(true) {
    chThdSleepMilliseconds(1000);
  }

}

#if 0

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
#endif

/* Get a free packet structure. */
struct PacketT *USBGetEmptyPacket(systime_t timeout)
{
  if(!g_comsInitDone)
    return 0;

  msg_t msg;
  if(chMBFetch(&g_emptyPackets,&msg,timeout) != MSG_OK)
    return 0;
  return (PacketT *)msg;
}



/* Post full packet. */
bool USBPostPacket(struct PacketT *pkt)
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

  if((pkt = USBGetEmptyPacket(TIME_IMMEDIATE)) == 0) {
    g_usbDropCount++;
    return false;
  }

  pkt->m_len = len;
  memcpy(&pkt->m_data,buff,len);
  return USBPostPacket(pkt);
}



void InitComs()
{
  if(g_comsInitDone)
    return ;

  chMBObjectInit(&g_emptyPackets,g_emptyPacketData,PACKET_QUEUE_SIZE);
  for(int i = 0;i < PACKET_QUEUE_SIZE;i++) {
    chMBPost(&g_emptyPackets,reinterpret_cast<msg_t>(&g_packetArray[i]),TIME_IMMEDIATE);
  }
  chMBObjectInit(&g_fullPackets,g_fullPacketData,PACKET_QUEUE_SIZE);

  g_comsInitDone = true;

  chThdCreateStatic(waThreadRxComs, sizeof(waThreadRxComs), NORMALPRIO+1, ThreadRxComs, NULL); // Prefer dealing with incoming messages.
#if 0
  chThdCreateStatic(waThreadTxComs, sizeof(waThreadTxComs), NORMALPRIO, ThreadTxComs, NULL);
#endif

}

void InitUSB(void)
{
  /*
   * Initialisation of USB is doen on the InitComs() rx thread to avoid a delay in starting up.
   * */
}

static void QueueTransmit(USBDriver *usbp, usbep_t ep) {

  static msg_t g_txMsg = 0;

  // Free last packet
  if(g_txMsg != 0) {
    if(chMBPostI(&g_emptyPackets,g_txMsg) != MSG_OK)
      g_usbErrorCount++;
    g_txMsg = 0;
  }

  // May need to transmit more than one packet at a time to get the required bandwidth.
  if(chMBFetchI(&g_fullPackets,&g_txMsg) == MSG_OK) {
    struct PacketT *packet = reinterpret_cast<struct PacketT *>(g_txMsg);
    usbStartTransmitI(usbp, ep, packet->m_data, packet->m_len);
  }

}

void bmcDataTransmitCallback(USBDriver *usbp, usbep_t ep)
{
  // Are we active ?
  if(!g_packetUSBActive)
    return ;

  osalSysLockFromISR();

  QueueTransmit(usbp,ep);

  osalSysUnlockFromISR();
}

void bmcDataReceivedCallback(USBDriver *usbp, usbep_t ep)
{
}

bool bmcRequestsHook(USBDriver *usbp)
{
  return false;
}

bool bmcSOFHookI(USBDriver *usbp, usbep_t epIn)
{
  /* If the USB driver is not in the appropriate state then transactions
     must not be started.*/
  if ((usbGetDriverStateI(usbp) != USB_ACTIVE) || ! g_packetUSBActive) {
    return true;
  }

  /* If there is already a transaction ongoing then another one cannot be
     started.*/
  if (usbGetTransmitStatusI(usbp,epIn)) {
    return true;
  }

  QueueTransmit(usbp,epIn);

  return true;
}

/* Allow transfers to be queued. */

void bmcWakeupHookI(USBDriver *usbp)
{
  g_packetUSBActive = true;
}

/* Make sure no transfers are queued. */

void bmcSuspendHookI(USBDriver *usbp)
{
  g_packetUSBActive = false;
}

/* Resetting the state of the subsystem.*/

void bmcConfigureHookI(USBDriver *usbp)
{

}

#endif
