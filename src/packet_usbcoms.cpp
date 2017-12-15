
#include "bmc.h"

#if USE_PACKETUSB

#include "packet_queue.hh"
#include "packet_usb.h"
#include "coms.h"
#include "hal.h"
#include "bmc.h"
#include <string.h>

bool g_packetTxUSBActive = false;
bool g_packetRxUSBActive = false;

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

static void QueueTransmitI(USBDriver *usbp, usbep_t ep) {

  static struct PacketT *g_txPkt = 0;
#if 1
  // Free last packet
  if(g_txPkt != 0) {
    g_txPacketQueue.ReturnEmptyPacketI(g_txPkt);
    g_txPkt = 0;
  }

  if((g_txPkt = g_txPacketQueue.FetchFullI()) != 0) {
    // May need to transmit more than one packet at a time to get the required bandwidth.
    usbStartTransmitI(usbp, ep, g_txPkt->m_data, g_txPkt->m_len);
  }
#endif
}

void bmcDataTransmitCallback(USBDriver *usbp, usbep_t ep)
{
  // Are we active ?
  if(!g_packetTxUSBActive)
    return ;

  osalSysLockFromISR();

  QueueTransmitI(usbp,ep);

  osalSysUnlockFromISR();
}


static struct PacketT *g_usbCurrentRxBuffer = 0;

static void startRecieveI(USBDriver *usbp, usbep_t ep)
{

  /* If the USB driver is not in the appropriate state then transactions
     must not be started.*/
  if ((usbGetDriverStateI(usbp) != USB_ACTIVE) || !g_packetRxUSBActive) {
    return ;
  }

  // Checking if there is already a transaction ongoing on the endpoint.
  if (usbGetReceiveStatusI(usbp, USBD1_DATA_OUT_EP)) {
    return ;
  }

  if(g_usbCurrentRxBuffer != 0) // Recieve in progress ??
    return ;

  /* Checking if there is a buffer ready for incoming data.*/

  struct PacketT *pkt = g_rxPacketQueue.GetEmptyPacketI();
  if(pkt == 0)
    return ; // No empty packets ready...

  g_usbCurrentRxBuffer = pkt;

  // Buffer found, starting a new transaction.
  usbStartReceiveI(usbp, ep,pkt->m_data, pkt->m_len);
}


void bmcDataReceivedCallback(USBDriver *usbp, usbep_t ep)
{
  osalSysLockFromISR();

  if(g_usbCurrentRxBuffer != 0) {
    g_rxPacketQueue.PostFullPacketI(g_usbCurrentRxBuffer);
    g_usbCurrentRxBuffer = 0;
  }

  startRecieveI(usbp,ep);

  osalSysUnlockFromISR();

}

bool bmcRequestsHook(USBDriver *usbp)
{
  return false;
}

bool bmcSOFHookI(USBDriver *usbp)
{
  /* If the USB driver is not in the appropriate state then transactions
     must not be started.*/
  if ((usbGetDriverStateI(usbp) != USB_ACTIVE) || ! g_packetTxUSBActive) {
    return true;
  }
  osalSysLockFromISR();

  /* If there is already a transaction ongoing then another one cannot be
     started.*/

  if (!usbGetTransmitStatusI(usbp,USBD1_DATA_IN_EP)) {
    QueueTransmitI(usbp,USBD1_DATA_IN_EP);
  }

  osalSysUnlockFromISR();

  return true;
}

/* Allow transfers to be queued. */

void bmcWakeupHookI(USBDriver *usbp)
{
  g_packetTxUSBActive = true;
  g_packetRxUSBActive = true;
}

/* Make sure no transfers are queued. */

void bmcSuspendHookI(USBDriver *usbp)
{
  g_packetTxUSBActive = false;
  g_packetRxUSBActive = false;
}

/* Resetting the state of the subsystem.*/

void bmcConfigureHookI(USBDriver *usbp)
{
  startRecieveI(usbp,USBD1_DATA_OUT_EP);
  g_packetTxUSBActive = true;
  g_packetRxUSBActive = true;
}

#endif
