
#include "bmc.h"

#if USE_PACKETUSB

#include "packet_queue.hh"
#include "packet_usb.h"
#include "coms.h"
#include "hal.h"
#include "bmc.h"
#include <string.h>

bool g_packetUSBActive = false;

PacketQueueC g_rxPacketQueue;

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
    struct PacketT *pkt = g_rxPacketQueue.FetchFull(TIME_INFINITE);
    if(pkt == 0) {
      // Failed, avoid a tight loop in case it repeats.
      chThdSleepMilliseconds(10);
      continue;
    }

    // Process the packet.
    ProcessPacket(pkt->m_data,pkt->m_len);

    // Return it to the list of empty packets.
    g_rxPacketQueue.ReturnEmptyPacketI(pkt);
  }

}


static void QueueTransmitIsoI(USBDriver *usbp) {

  static struct PacketT *g_txPkt = 0;
#if 1
  // Free last packet
  if(g_txPkt != 0) {
    g_txPacketQueue.ReturnEmptyPacketI(g_txPkt);
    g_txPkt = 0;
  }

  if((g_txPkt = g_txPacketQueue.FetchFullI()) != 0) {
    // May need to transmit more than one packet at a time to get the required bandwidth.
    usbStartTransmitI(usbp, USBD1_DATA_IN_EP, g_txPkt->m_data, g_txPkt->m_len);
  }
#endif
}

static void QueueTransmitIntrI(USBDriver *usbp) {

  static struct PacketT *g_txPkt = 0;
#if 1
  // Free last packet
  if(g_txPkt != 0) {
    g_txIntrPacketQueue.ReturnEmptyPacketI(g_txPkt);
    g_txPkt = 0;
  }

  if((g_txPkt = g_txIntrPacketQueue.FetchFullI()) != 0) {
    // May need to transmit more than one packet at a time to get the required bandwidth.
    usbStartTransmitI(usbp, USBD1_INTR_IN_EP, g_txPkt->m_data, g_txPkt->m_len);
  }
#endif
}

void bmcDataTransmitCallback(USBDriver *usbp, usbep_t ep)
{
  (void)ep;

  // Are we active ?
  if(!g_packetUSBActive)
    return ;

  osalSysLockFromISR();

  QueueTransmitIsoI(usbp);

  osalSysUnlockFromISR();
}


static struct PacketT *g_usbCurrentDataRxBuffer = 0;

static void startRecieveDataI(USBDriver *usbp)
{

  /* If the USB driver is not in the appropriate state then transactions
     must not be started.*/
  if ((usbGetDriverStateI(usbp) != USB_ACTIVE) || !g_packetUSBActive) {
    return ;
  }

  // Checking if there is already a transaction ongoing on the endpoint.
  if (usbGetReceiveStatusI(usbp, USBD1_DATA_OUT_EP)) {
    // End point is already receiving.
    return ;
  }

  if(g_usbCurrentDataRxBuffer != 0) // Receive in progress ??
    return ;

  /* Checking if there is a buffer ready for incoming data.*/

  struct PacketT *pkt = g_rxPacketQueue.GetEmptyPacketI();
  if(pkt == 0)
    return ; // No empty packets ready...

  g_usbCurrentDataRxBuffer = pkt;

  // Buffer found, starting a new transaction.
  usbStartReceiveI(usbp, USBD1_DATA_OUT_EP,pkt->m_data, BMC_MAXPACKETSIZE);
}



void bmcDataReceivedCallback(USBDriver *usbp, usbep_t ep)
{
  (void)ep;

  osalSysLockFromISR();

  if(g_usbCurrentDataRxBuffer != 0) {
    g_usbCurrentDataRxBuffer->m_len = usbGetReceiveTransactionSizeX(usbp, ep);
    g_rxPacketQueue.PostFullPacketI(g_usbCurrentDataRxBuffer);
    g_usbCurrentDataRxBuffer = 0;
  }

  startRecieveDataI(usbp);

  osalSysUnlockFromISR();

}

void bmcIntrTransmitCallback(USBDriver *usbp, usbep_t ep)
{
  (void)ep;

  // Are we active ?
  if(!g_packetUSBActive)
    return ;

  osalSysLockFromISR();

  QueueTransmitIntrI(usbp);

  osalSysUnlockFromISR();
}

static struct PacketT *g_usbCurrentIntrRxBuffer = 0;

static void startRecieveIntrI(USBDriver *usbp)
{

  /* If the USB driver is not in the appropriate state then transactions
     must not be started.*/
  if ((usbGetDriverStateI(usbp) != USB_ACTIVE) || !g_packetUSBActive) {
    return ;
  }

  // Checking if there is already a transaction ongoing on the endpoint.
  if (usbGetReceiveStatusI(usbp, USBD1_INTR_OUT_EP)) {
    return ;
  }

  if(g_usbCurrentIntrRxBuffer != 0) // Receive in progress ??
    return ;

  /* Checking if there is a buffer ready for incoming data.*/

  struct PacketT *pkt = g_rxPacketQueue.GetEmptyPacketI();
  if(pkt == 0)
    return ; // No empty packets ready...

  g_usbCurrentIntrRxBuffer = pkt;

  // Buffer found, starting a new transaction.
  usbStartReceiveI(usbp, USBD1_INTR_OUT_EP,pkt->m_data, BMC_MAXPACKETSIZE);
}


void bmcIntrReceivedCallback(USBDriver *usbp, usbep_t ep)
{
  (void) ep;

  osalSysLockFromISR();

  if(g_usbCurrentIntrRxBuffer != 0) {
    g_usbCurrentIntrRxBuffer->m_len = usbGetReceiveTransactionSizeX(usbp, ep);
    g_rxPacketQueue.PostFullPacketI(g_usbCurrentIntrRxBuffer);
    g_usbCurrentIntrRxBuffer = 0;
  }

  startRecieveIntrI(usbp);

  osalSysUnlockFromISR();
}


bool bmcRequestsHook(USBDriver *usbp)
{
  (void) usbp;
  return false;
}

bool bmcSOFHookI(USBDriver *usbp)
{
  /* If the USB driver is not in the appropriate state then transactions
     must not be started.*/
  if ((usbGetDriverStateI(usbp) != USB_ACTIVE) || ! g_packetUSBActive) {
    return true;
  }
  osalSysLockFromISR();

  /* If there is already a transaction ongoing then another one cannot be
     started.*/

  if (!usbGetTransmitStatusI(usbp,USBD1_DATA_IN_EP)) { // Returns true if transmitting.
    QueueTransmitIsoI(usbp);
  }

  if (!usbGetTransmitStatusI(usbp,USBD1_INTR_IN_EP)) { // Returns true if transmitting.
    QueueTransmitIntrI(usbp);
  }

  osalSysUnlockFromISR();

  return true;
}

/* Allow transfers to be queued. */

void bmcWakeupHookI(USBDriver *usbp)
{
  (void) usbp;
  g_packetUSBActive = true;
}

/* Make sure no transfers are queued. */

void bmcSuspendHookI(USBDriver *usbp)
{
  (void) usbp;
  g_packetUSBActive = false;
}

/* Resetting the state of the subsystem.*/

void bmcConfigureHookI(USBDriver *usbp)
{
  g_packetUSBActive = true;
  startRecieveDataI(usbp);
  startRecieveIntrI(usbp);
}

void InitComs()
{
  if(g_comsInitDone)
    return ;

  g_comsInitDone = true;

  chThdCreateStatic(waThreadRxComs, sizeof(waThreadRxComs), NORMALPRIO+1, ThreadRxComs, NULL); // Prefer dealing with incoming messages.
}

void InitUSB(void)
{
  /*
   * Initialisation of USB is doen on the InitComs() rx thread to avoid a delay in starting up.
   * */
}

#endif
