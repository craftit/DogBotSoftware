
#include "bmc.h"

#if USE_PACKETUSB

#include "packet_queue.hh"
#include "packet_usb.h"
#include "canbus.h"
#include "coms.h"
#include "hal.h"
#include "bmc.h"
#include <string.h>

bool g_packetUSBActive = false;

PacketQueueC g_rxPacketQueue;

static THD_WORKING_AREA(waThreadRxComs, 1024);
static THD_FUNCTION(ThreadRxComs, arg) {

  (void)arg;
  chRegSetThreadName("rxPacketComs");

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(&USBD2);
  chThdSleepMilliseconds(1500);
  usbStart(&USBD2,&usbcfg);
  usbConnectBus(&USBD2);

  while(true) {
    struct PacketT *pkt = g_rxPacketQueue.FetchFull(TIME_INFINITE);
    if(pkt == 0) {
      // Failed, avoid a tight loop in case it repeats.
      chThdSleepMilliseconds(10);
      continue;
    }

    int at = 0;
    while(at < pkt->m_len) {
      int len = pkt->m_data[at++];
      if((at+len) > pkt->m_len) {
        USBSendError(g_deviceId,CET_InternalError,0,1);
        g_usbErrorCount++;
        at = pkt->m_len; // Suppress a double error being reported after we exit the loop.
        break;
      }
      // Process the packet.
      ProcessPacket(&(pkt->m_data[at]),len);
      at += len;
    }
    if(at != pkt->m_len) {
      USBSendError(g_deviceId,CET_InternalError,0,2);
      g_usbErrorCount++;
    }
    // Return it to the list of empty packets.
    g_rxPacketQueue.ReturnEmptyPacketI(pkt);
  }

}


static void QueueTransmitIsoI(USBDriver *usbp) {


  // Wrap up a set of packets to send.
  static uint8_t txBuffer[64];
  int at = 0;
  while(true) {
    struct PacketT *txPkt = g_txPacketQueue.FetchFullI();
    if(txPkt == 0)
      break;
    if(txPkt->m_len == 0) {
      g_usbErrorCount++;
      g_txPacketQueue.ReturnEmptyPacketI(txPkt);
      continue;
    }
    if((at + txPkt->m_len + 1) >= 64) {
      // We shouldn't overflow the buffer as all the packets should be less than 14 bytes, but flag a problem and drop the packet.
      g_usbErrorCount++;
      g_txPacketQueue.ReturnEmptyPacketI(txPkt);
      break;
    }
    txBuffer[at++] = txPkt->m_len;
    memcpy(&txBuffer[at],txPkt->m_data,txPkt->m_len);
    at += txPkt->m_len;
    g_txPacketQueue.ReturnEmptyPacketI(txPkt);
    if(at > 50) // Getting full ?
      break;
  }
  // Anything to send ?
  if(at == 0)
    return ;
  usbStartTransmitI(usbp, USBD2_DATA_IN_EP, txBuffer,at);
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
  if (usbGetReceiveStatusI(usbp, USBD2_DATA_OUT_EP)) {
    // End point is already receiving.
    return ;
  }

  if(g_usbCurrentDataRxBuffer != 0) // Receive in progress ??
    return ;

  /* Checking if there is a buffer ready for incoming data.*/

  g_usbCurrentDataRxBuffer = g_rxPacketQueue.GetEmptyPacketI();
  if(g_usbCurrentDataRxBuffer == 0)
    return ; // No empty packets ready...

  // Buffer found, starting a new transaction.
  usbStartReceiveI(usbp, USBD2_DATA_OUT_EP,g_usbCurrentDataRxBuffer->m_data, BMC_MAXPACKETSIZE);
}



void bmcDataReceivedCallback(USBDriver *usbp, usbep_t ep)
{
  (void)ep;

  osalSysLockFromISR();

  if(g_usbCurrentDataRxBuffer != 0) {
    int len = usbGetReceiveTransactionSizeX(usbp, ep);
    if(len <= 0) {
      g_rxPacketQueue.ReturnEmptyPacketI(g_usbCurrentDataRxBuffer);
      g_usbCurrentDataRxBuffer = 0;
    } else {
      g_usbCurrentDataRxBuffer->m_len = len;
      g_rxPacketQueue.PostFullPacketI(g_usbCurrentDataRxBuffer);
      g_usbCurrentDataRxBuffer = 0;
    }
  }

  startRecieveDataI(usbp);

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

  if (!usbGetTransmitStatusI(usbp,USBD2_DATA_IN_EP)) { // Returns true if transmitting.
    QueueTransmitIsoI(usbp);
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
}

void InitComs()
{
  if(g_comsInitDone)
    return ;

  g_comsInitDone = true;

  chThdCreateStatic(waThreadRxComs, sizeof(waThreadRxComs), NORMALPRIO, ThreadRxComs, NULL);
}

void InitUSB(void)
{
  /*
   * Initialisation of USB is done on the InitComs() rx thread to avoid a delay in starting up.
   * */
}

#endif
